using System;
using System.Collections;               // para IEnumerator (corutina)
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.InputSystem;

[DisallowMultipleComponent]
public class QuaternionReceiverWithInitial : MonoBehaviour
{
    [Header("Configuración de red")]
    public int port = 25001;

    [Header("Humanoid / Mixamo")]
    [Tooltip("Animator del modelo Humanoid que quieres controlar")]
    public Animator animator;

    // === Mapeo 3 IMUs → huesos ===
    public enum BoneChoice { Biceps, Antebrazo, Mano }

    [Serializable]
    public class ImuBinding
    {
        [Tooltip("Nombre tal como llega por TCP (p.ej., GROOT_01)")]
        public string sensorName;

        [Tooltip("Hueso objetivo del brazo derecho")]
        public BoneChoice target = BoneChoice.Antebrazo;

        [HideInInspector] public string debugLastLine;   // opcional, solo inspección visual
    }

    [Header("Asignación de IMUs (3 slots)")]
    public ImuBinding[] bindings = new ImuBinding[3]
    {
        new ImuBinding(){ sensorName = "GROOT_01", target = BoneChoice.Biceps     },
        new ImuBinding(){ sensorName = "GROOT_02", target = BoneChoice.Antebrazo  },
        new ImuBinding(){ sensorName = "GROOT_03", target = BoneChoice.Mano       },
    };

    [Header("Debug")]
    [Tooltip("Muestra un panel con el estado ON/OFF de cada IMU configurada")]
    public bool showStatus = true;

    [Header("Ángulos anatómicos (deg) — brazo derecho")]
    public Vector3 shoulderEulerDeg; // Hombro (RightUpperArm local)
    public Vector3 elbowEulerDeg;    // Codo   (RightLowerArm local)
    public Vector3 wristEulerDeg;    // Muñeca (RightHand local)

    [Header("Medición de latencia")]
    [Tooltip("Imprimir el promedio de latencia al salir de la aplicación")]
    public bool summaryOnQuit = true;

    // === Latencia (RX -> Apply) ===
    private static readonly double _ticksToMs = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
    private double _latencySumMs = 0.0;
    private long _latencySamples = 0;

    // --- Estado red/parseo ---
    private CancellationTokenSource _cts;
    private Task _listenerTask;
    private TcpListener listener;

    static readonly NumberFormatInfo _nfi = CultureInfo.InvariantCulture.NumberFormat;

    // --- Rig/huesos y padres ---
    private Transform _boneUpperArm, _boneLowerArm, _boneHand;
    private Transform _parentUpperArm, _parentLowerArm, _parentHand;

    // Rotación en MUNDO en T-Pose (reposo) de cada hueso
    private Quaternion _restWorldUpperArm, _restWorldLowerArm, _restWorldHand;

    private bool _rigReady = false;

    // === Estado por sensor (nombre) ===
    private class SensorState
    {
        // Mapeo en MUNDO: R_bone_world = worldMap * R_sensor_world
        public Quaternion worldMap = Quaternion.identity;
        public bool worldCalibrated = false;

        public Quaternion latestSensorWorld = Quaternion.identity; // Rs_t (tras SensorToUnity)
        public Quaternion latestBoneWorld   = Quaternion.identity;  // objetivo en mundo: M * Rs_t

        // sello temporal al recibir (ticks de Stopwatch)
        public long lastRxTicks;
    }

    private readonly object _lock = new();
    private readonly Dictionary<string, SensorState> _sensors = new();

    // ===== Convención sensor → Unity (base) =====
    static Quaternion SensorToUnity(Quaternion qS)
    {
        var qSwap = new Quaternion(qS.x, qS.z, qS.y, qS.w);
        qSwap.x = -qSwap.x;
        qSwap.z = -qSwap.z;
        qSwap.w = -qSwap.w;
        return Quaternion.Normalize(qSwap);
    }

    private IEnumerator Start()
    {
        if (animator == null)
        {
            Debug.LogError("[QuaternionReceiver] Asigna un Animator (Humanoid).");
            enabled = false;
            yield break;
        }

        // Resolver huesos del brazo derecho
        _boneUpperArm = animator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        _boneLowerArm = animator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        _boneHand     = animator.GetBoneTransform(HumanBodyBones.RightHand);

        if (_boneUpperArm == null || _boneLowerArm == null || _boneHand == null)
        {
            Debug.LogError("[QuaternionReceiver] No se pudieron resolver todos los huesos del brazo derecho.");
            enabled = false;
            yield break;
        }

        _parentUpperArm = _boneUpperArm.parent;
        _parentLowerArm = _boneLowerArm.parent;
        _parentHand     = _boneHand.parent;

        // Forzar T-Pose 1 frame para capturar reposo en MUNDO
        var saved = animator.runtimeAnimatorController;
        animator.runtimeAnimatorController = null;
        animator.Update(0f);

        _restWorldUpperArm = _boneUpperArm.rotation;
        _restWorldLowerArm = _boneLowerArm.rotation;
        _restWorldHand     = _boneHand.rotation;

        yield return null; // un frame en T-Pose

        animator.runtimeAnimatorController = saved;
        _rigReady = true;

        // Listener en background
        _cts = new CancellationTokenSource();
        _listenerTask = Task.Run(() => ListenAsync(_cts.Token), _cts.Token);
    }

    private async Task ListenAsync(CancellationToken token)
    {
        try
        {
            listener = new TcpListener(IPAddress.Any, port);
            listener.Start();
            Debug.Log($"[QuaternionReceiver] Escuchando en 127.0.0.1:{port}");

            using (var client = await listener.AcceptTcpClientAsync().ConfigureAwait(false))
            using (var reader = new StreamReader(client.GetStream()))
            {
                Debug.Log("[QuaternionReceiver] Cliente conectado");

                while (!token.IsCancellationRequested && !reader.EndOfStream)
                {
                    var raw = await reader.ReadLineAsync().ConfigureAwait(false);
                    if (string.IsNullOrWhiteSpace(raw)) continue;

                    var line = raw.Trim();
                    if (line.StartsWith("\ufeff")) line = line.TrimStart('\ufeff');

                    var parts = line.Split(',');
                    if (parts.Length < 4)
                    {
                        Debug.LogWarning($"[QuaternionReceiver] Línea inválida ({parts.Length} cols): \"{line}\"");
                        continue;
                    }

                    int offsetCols = parts.Length - 4;
                    string name = (offsetCols > 0)
                                  ? string.Join(",", parts, 0, offsetCols).Trim()
                                  : string.Empty;

                    if (!float.TryParse(parts[offsetCols + 0], NumberStyles.Float, _nfi, out var qx) ||
                        !float.TryParse(parts[offsetCols + 1], NumberStyles.Float, _nfi, out var qy) ||
                        !float.TryParse(parts[offsetCols + 2], NumberStyles.Float, _nfi, out var qz) ||
                        !float.TryParse(parts[offsetCols + 3], NumberStyles.Float, _nfi, out var qw))
                    {
                        Debug.LogWarning($"[QuaternionReceiver] PARSE FAIL en \"{line}\"");
                        continue;
                    }

                    var qSensorWorld = SensorToUnity(new Quaternion(qx, qy, qz, qw));

                    lock (_lock)
                    {
                        if (!_sensors.TryGetValue(name, out var st))
                        {
                            st = new SensorState();
                            _sensors[name] = st; // ON para ese nombre
                        }

                        st.latestSensorWorld = qSensorWorld;

                        // ¿A qué hueso está ligado este nombre?
                        if (!TryGetBoneForName(name, out var targetBone, out var restWorld, out var parentTf))
                        {
                            // Aún sin binding: solo guardamos última Rs_t
                            continue;
                        }

                        // Calibración en MUNDO por sensor: M = R_bone_rest_world * Rs_0^-1
                        if (!st.worldCalibrated)
                        {
                            st.worldMap = restWorld * Quaternion.Inverse(qSensorWorld);
                            st.worldCalibrated = true;
                        }

                        // Objetivo en mundo para este sensor/hueso
                        st.latestBoneWorld = st.worldMap * qSensorWorld;

                        // sello temporal para latencia
                        st.lastRxTicks = System.Diagnostics.Stopwatch.GetTimestamp();

                        // Actualizar binding debug (opcional)
                        for (int i = 0; i < bindings.Length; i++)
                        {
                            if (!string.IsNullOrEmpty(bindings[i].sensorName) &&
                                string.Equals(bindings[i].sensorName, name, StringComparison.Ordinal))
                            {
                                bindings[i].debugLastLine = line;
                            }
                        }
                    }
                }
            }
        }
        catch (ObjectDisposedException) { /* cierre ordenado */ }
        catch (Exception ex)
        {
            Debug.LogError($"[QuaternionReceiver] Socket error: {ex.Message}");
        }
    }

    private void Update()
    {
        if (!_rigReady) return;

        // Recalibración global: cada IMU fijará su M con su SIGUIENTE muestra
        if (Keyboard.current != null && Keyboard.current.cKey.wasPressedThisFrame)
        {
            lock (_lock)
            {
                foreach (var kv in _sensors)
                    kv.Value.worldCalibrated = false;
            }
        }

        // Reportar promedio manual (L) y resetear
        if (Keyboard.current != null && Keyboard.current.lKey.wasPressedThisFrame)
        {
            LogAndResetLatencySummary();
        }

        // 1) Resolver world targets snapshot (si existen) para cada hueso + ticks de RX
        Quaternion? worldUpper = null, worldLower = null, worldHand = null;
        long rxTicksUpper = 0, rxTicksLower = 0, rxTicksHand = 0;

        lock (_lock)
        {
            for (int i = 0; i < bindings.Length; i++)
            {
                var b = bindings[i];
                if (string.IsNullOrEmpty(b.sensorName)) continue;
                if (!_sensors.TryGetValue(b.sensorName, out var st)) continue;
                if (!st.worldCalibrated) continue;

                switch (b.target)
                {
                    case BoneChoice.Biceps:
                        worldUpper = st.latestBoneWorld;
                        rxTicksUpper = st.lastRxTicks;
                        break;
                    case BoneChoice.Antebrazo:
                        worldLower = st.latestBoneWorld;
                        rxTicksLower = st.lastRxTicks;
                        break;
                    case BoneChoice.Mano:
                        worldHand  = st.latestBoneWorld;
                        rxTicksHand = st.lastRxTicks;
                        break;
                }
            }
        }

        // 2) Aplicar de PADRE a HIJO para evitar dobles: UpperArm → LowerArm → Hand
        bool appliedUpper = false, appliedLower = false, appliedHand = false;

        // BÍCEPS
        if (worldUpper.HasValue)
        {
            var parentWorld = _parentUpperArm ? _parentUpperArm.rotation : Quaternion.identity;
            var local = Quaternion.Inverse(parentWorld) * worldUpper.Value;
            _boneUpperArm.localRotation = local;
            appliedUpper = true;
        }

        // ANTEBRAZO
        if (worldLower.HasValue)
        {
            var parentWorld = _parentLowerArm ? _parentLowerArm.rotation : Quaternion.identity;
            var local = Quaternion.Inverse(parentWorld) * worldLower.Value;
            _boneLowerArm.localRotation = local;
            appliedLower = true;
        }

        // MANO
        if (worldHand.HasValue)
        {
            var parentWorld = _parentHand ? _parentHand.rotation : Quaternion.identity;
            var local = Quaternion.Inverse(parentWorld) * worldHand.Value;
            _boneHand.localRotation = local;
            appliedHand = true;
        }

        // 3) Acumular latencia (RX -> Apply) sin imprimir por frame
        var nowTicks = System.Diagnostics.Stopwatch.GetTimestamp();
        if (appliedUpper && rxTicksUpper != 0)
        {
            _latencySumMs += (nowTicks - rxTicksUpper) * _ticksToMs;
            _latencySamples++;
        }
        if (appliedLower && rxTicksLower != 0)
        {
            _latencySumMs += (nowTicks - rxTicksLower) * _ticksToMs;
            _latencySamples++;
        }
        if (appliedHand && rxTicksHand != 0)
        {
            _latencySumMs += (nowTicks - rxTicksHand) * _ticksToMs;
            _latencySamples++;
        }

        // 4) Capturar ángulos anatómicos (Euler) si hubo actualización
        if (appliedUpper || appliedLower || appliedHand)
        {
            CaptureRightArmAngles();
        }
    }

    // ====== Verificador simple en pantalla ======
    private void OnGUI()
    {
        if (!showStatus || bindings == null) return;

        const int margin = 10;
        const int lineH  = 20;

        GUI.Box(new Rect(margin, margin, 260, 24 + lineH * bindings.Length), "IMUs");

        float y = margin + 24;
        lock (_lock)
        {
            foreach (var b in bindings)
            {
                if (string.IsNullOrEmpty(b.sensorName)) continue;
                bool on = _sensors.ContainsKey(b.sensorName);
                string text = $"{b.sensorName}: {(on ? "ON" : "OFF")}";
                GUI.Label(new Rect(margin + 8, y, 240, lineH), text);
                y += lineH;
            }
        }
    }

    private void OnApplicationQuit()
    {
        if (summaryOnQuit)
        {
            PrintLatencySummary();
        }

        listener?.Stop();
        _cts?.Cancel();
        try { _listenerTask?.Wait(100); } catch { /* ignore */ }
    }

    // --- Utilidades ---
    private bool TryGetBoneForName(string sensorName, out Transform bone, out Quaternion restWorld, out Transform parentTf)
    {
        bone = null; restWorld = Quaternion.identity; parentTf = null;

        // Busca el binding activo que use ese nombre
        for (int i = 0; i < bindings.Length; i++)
        {
            if (!string.Equals(bindings[i].sensorName, sensorName, StringComparison.Ordinal))
                continue;

            switch (bindings[i].target)
            {
                case BoneChoice.Biceps:
                    bone = _boneUpperArm; restWorld = _restWorldUpperArm; parentTf = _parentUpperArm; return true;
                case BoneChoice.Antebrazo:
                    bone = _boneLowerArm; restWorld = _restWorldLowerArm; parentTf = _parentLowerArm; return true;
                case BoneChoice.Mano:
                    bone = _boneHand;     restWorld = _restWorldHand;     parentTf = _parentHand;     return true;
            }
        }
        return false;
    }

    // Captura de ángulos anatómicos (Euler) del brazo derecho
    private void CaptureRightArmAngles()
    {
        shoulderEulerDeg = _boneUpperArm.localRotation.eulerAngles;
        elbowEulerDeg    = _boneLowerArm.localRotation.eulerAngles;
        wristEulerDeg    = _boneHand.localRotation.eulerAngles;
        // (No se imprime: quedan visibles en el Inspector)
    }

    // ---- Latencia: helpers ----
    private void LogAndResetLatencySummary()
    {
        PrintLatencySummary();
        _latencySumMs = 0.0;
        _latencySamples = 0;
    }

    private void PrintLatencySummary()
    {
        if (_latencySamples > 0)
        {
            double avg = _latencySumMs / _latencySamples;
            Debug.Log($"Latencia media (RX→Apply) entre todos los sensores: {avg:F1} ms  | muestras={_latencySamples}");
        }
        else
        {
            Debug.Log("Latencia media: sin muestras registradas.");
        }
    }
}
