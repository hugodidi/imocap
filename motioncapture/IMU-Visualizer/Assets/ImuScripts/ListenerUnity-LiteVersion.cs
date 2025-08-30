using System;
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

    // ─────────────────────────────────────────────────────────────
    [Header("Rotación inicial (en grados)")] 
    [Tooltip("Ángulos Euler XYZ que quieras que tenga el objeto al arrancar.")]
    public Vector3 initialEuler = Vector3.zero;

    // Si prefieres asignar directamente un Quaternion, usa esto en su lugar:
    // public Quaternion initialQuat = Quaternion.identity;

    // ─────────────────────────────────────────────────────────────
    private CancellationTokenSource _cts;
    private Task _listenerTask;
    private TcpListener listener;

    readonly object _rotLock = new();
    private Quaternion _latestRaw;       
    private volatile bool _hasNewRotation = false;

    private Quaternion _offset = Quaternion.identity;
    private bool _calibrated = false;
    // NUEVO: último nombre de sensor recibido (p.ej., "GROOT_01")
    private string _lastSensorName = string.Empty;
    


    static readonly NumberFormatInfo _nfi = CultureInfo.InvariantCulture.NumberFormat;

    // ─────────────────────────────────────────────────────────────
    private void Start()
    {
        // ① Asigno la rotación inicial que quiero en Runtime:
        //    Aquí convierto los Euler que he puesto en el Inspector a Quaternion.
        transform.rotation = Quaternion.Euler(initialEuler);

        // … o, si usas directamente initialQuat:
        // transform.rotation = initialQuat;

        // ② Luego arranco el listener en un Task separado.
        _cts = new CancellationTokenSource();
        _listenerTask = Task.Run(() => ListenAsync(_cts.Token), _cts.Token);
    }

    // ─────────────────────────────────────────────────────────────
    private async Task ListenAsync(CancellationToken token)
    {
        try
        {
            listener = new TcpListener(IPAddress.Any, port);
            listener.Start();
            Debug.Log($"[QuaternionReceiver] Escuchando en 127.0.0.1:{port}");

            while (!token.IsCancellationRequested)
            {
                var client = await listener.AcceptTcpClientAsync();
                Debug.Log("[QuaternionReceiver] Cliente conectado");

                using (var reader = new StreamReader(client.GetStream()))
                {
                    while (!token.IsCancellationRequested && !reader.EndOfStream)
                    {
                        var line = await reader.ReadLineAsync();
                        if (string.IsNullOrWhiteSpace(line)) continue;

                        var parts = line.Split(',');
                        if (parts.Length != 4 && parts.Length != 5) continue;

                        // Si viene con nombre, lo guardamos y desplazamos el índice de parseo
                        int o = 0;
                        if (parts.Length == 5)
                        {
                            _lastSensorName = parts[0].Trim();  // <-- aquí queda almacenado el nombre, p.ej. "GROOT_01"
                            // justo después de asignar _lastSensorName
                            Debug.Log($"[QuaternionReceiver] Sensor activo: {_lastSensorName}");

                            o = 1;

                        }

                        if (float.TryParse(parts[o + 0], NumberStyles.Float, _nfi, out var x) &&
                            float.TryParse(parts[o + 1], NumberStyles.Float, _nfi, out var y) &&
                            float.TryParse(parts[o + 2], NumberStyles.Float, _nfi, out var z) &&
                            float.TryParse(parts[o + 3], NumberStyles.Float, _nfi, out var w))
                        {
                            // Cuaternión crudo del sensor:
                            var qRaw = new Quaternion(x, y, z, w);
                            // Mapeo al sistema de Unity:
                            var qUnityMapped = SensorToUnity(qRaw);

                            // Si no está calibrado, guardo el offset:
                            if (!_calibrated)
                            {
                                _offset = Quaternion.Inverse(qUnityMapped);
                                _calibrated = true;
                            }

                            // Aplico offset:
                            var qFinal = _offset * qUnityMapped;

                            lock (_rotLock)
                            {
                                _latestRaw = qFinal;
                                _hasNewRotation = true;
                            }

                            // Para debug:
                            // Debug.Log($"PARSE OK RAW▶ {qRaw.eulerAngles}  → MAPPED▶ {qUnityMapped.eulerAngles}  → FINAL▶{qFinal.eulerAngles}");
                        }
                        else
                        {
                            Debug.LogWarning($"[QuaternionReceiver] PARSE FAIL en \"{line}\"");
                        }
                    }
                }
            }
        }
        catch (ObjectDisposedException) { /* listener.Stop() */ }
        catch (Exception ex)
        {
            Debug.LogError($"[QuaternionReceiver] Socket error: {ex.Message}");
            // ② Luego arranco el listener en un Task separado.
            _cts = new CancellationTokenSource(); 
            _listenerTask = Task.Run(() => ListenAsync(_cts.Token), _cts.Token);
        }
    }

    // ─────────────────────────────────────────────────────────────
    private void Update()
    {
        // Para recalibrar en cualquier momento con la tecla C:
        if (Keyboard.current != null && Keyboard.current.cKey.wasPressedThisFrame)
            _calibrated = false;

        if (_hasNewRotation)
        {
            lock (_rotLock)
            {
                // ⑤ A partir de aquí estoy sobreescribiendo la rotación que puse en Start()
                //    Si en cambio quisiera “sumar” la rotación del sensor a la rotación inicial,
                //    podría hacer: transform.rotation *= _latestRaw;
                transform.rotation = _latestRaw;
                _hasNewRotation = false;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────
    private async void OnApplicationQuit()
    {
        listener?.Stop();
        _cts.Cancel();
        try { await _listenerTask; }
        catch (OperationCanceledException) { }
    }

    // ─────────────────────────────────────────────────────────────
    // Función que convierte del sistema del sensor (Z arriba, Y adelante, X al lado)
    // al sistema de coordenadas diestro de Unity (Y arriba, Z adelante, X derecha).
    static Quaternion SensorToUnity(Quaternion qS)
    {
        // 1) Reordenar ejes: (x_s, y_s, z_s) → (x_u, y_u, z_u)
        //    X_s → X_u,  Y_s → Z_u,  Z_s → Y_u
        var qSwap = new Quaternion(qS.x, qS.z, qS.y, qS.w);

        // 2) Cambiar de mano diestro (sensor) a zurdo (Unity):
        //    Invertimos X, Z y W (sigue siendo un número impar de inversiones).
        qSwap.x = -qSwap.x;
        qSwap.z = -qSwap.z;
        qSwap.w = -qSwap.w;

        // 3) Normalizamos para corregir drift
        return Quaternion.Normalize(qSwap);
    }
}

