package com.example.wearablehmsmobileapp

import android.Manifest
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.pm.PackageManager
import android.os.*
import android.util.Log
import android.view.View
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.example.wearablehmsmobileapp.databinding.ActivityMainBinding
import org.json.JSONObject
import java.nio.charset.Charset
import java.util.*

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private val mainHandler = Handler(Looper.getMainLooper())

    // ---- Replace these with your device details ----
    private val TARGET_DEVICE_NAME = "WearableHMS" // e.g., your peripheral's name
    private val SERVICE_UUID = UUID.fromString("0000xxxx-0000-1000-8000-00805f9b34fb")
    private val CHARACTERISTIC_UUID = UUID.fromString("0000yyyy-0000-1000-8000-00805f9b34fb")
    // ------------------------------------------------

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothGatt: BluetoothGatt? = null
    private var isScanning = false

    // Simple thresholds (tune freely)
    private object Thresholds {
        const val HR_HIGH = 100.0
        const val SPO2_LOW = 92.0
        const val TEMP_HIGH = 37.8   // if using skin temp, adjust accordingly
        const val TEMP_LOW  = 35.0
        const val STRESS_HIGH = 70.0
        const val HRV_RMSSD_LOW = 20.0
    }

    // Runtime permissions (Android 12+)
    private val blePermsLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { perms ->
        val granted = perms.values.all { it }
        if (granted) startScan() else showAlert("Bluetooth permissions denied")
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        bluetoothAdapter = (getSystemService(BluetoothManager::class.java)).adapter

        requestBlePermissions()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopScan()
        bluetoothGatt?.close()
    }

    // --- Permissions & scanning ---
    private fun requestBlePermissions() {
        val needs = mutableListOf<String>()
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED)
            needs += Manifest.permission.BLUETOOTH_SCAN
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED)
            needs += Manifest.permission.BLUETOOTH_CONNECT
        // Some devices still need location for scans to work reliably
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED)
            needs += Manifest.permission.ACCESS_FINE_LOCATION

        if (needs.isNotEmpty()) blePermsLauncher.launch(needs.toTypedArray())
        else startScan()
    }

    private fun startScan() {
        if (isScanning) return
        val scanner = bluetoothAdapter?.bluetoothLeScanner ?: return
        val filters = listOf(ScanFilter.Builder().setDeviceName(TARGET_DEVICE_NAME).build())
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()

        isScanning = true
        scanner.startScan(filters, settings, scanCallback)
        showAlert("Scanning for $TARGET_DEVICE_NAME…")
        mainHandler.postDelayed({ stopScan() }, 15000) // safety timeout
    }

    private fun stopScan() {
        if (!isScanning) return
        val scanner = bluetoothAdapter?.bluetoothLeScanner ?: return
        scanner.stopScan(scanCallback)
        isScanning = false
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            if (result.device.name == TARGET_DEVICE_NAME) {
                stopScan()
                connectGatt(result.device)
            }
        }
    }

    private fun connectGatt(device: BluetoothDevice) {
        showAlert("Connecting to ${device.name}…")
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) return
        bluetoothGatt = device.connectGatt(this, false, gattCallback)
    }

    // --- GATT & notifications ---
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                runOnUiThread { showAlert("Discovering services…") }
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                runOnUiThread { showAlert("Disconnected") }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            val service = gatt.getService(SERVICE_UUID)
            val ch = service?.getCharacteristic(CHARACTERISTIC_UUID)

            if (service == null || ch == null) {
                runOnUiThread { showAlert("Service/Characteristic not found") }
                return
            }
            enableNotifications(gatt, ch)
            runOnUiThread { showAlert("Connected • Waiting for data…") }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            val bytes = characteristic.value ?: return
            val jsonStr = bytes.toString(Charset.forName("UTF-8")).trim()
            Log.d("BLE_JSON", jsonStr)
            handleIncomingJson(jsonStr)
        }
    }

    private fun enableNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) return
        gatt.setCharacteristicNotification(characteristic, true)
        val cccd = characteristic.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
        cccd?.let {
            it.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            gatt.writeDescriptor(it)
        }
    }

    // --- JSON handling & UI updates ---
    private fun handleIncomingJson(jsonStr: String) {
        try {
            val obj = JSONObject(jsonStr)
            val reading = Reading(
                cycleId   = obj.optInt("cycle_id"),
                timestamp = obj.optLong("timestamp"),
                bpm       = obj.optDouble("bpm"),
                spo2      = obj.optDouble("spo2"),
                temp      = obj.optDouble("temp"),
                gsr       = obj.optDouble("gsr"),
                hrvRmssd  = obj.optDouble("hrv_rmssd"),
                hrvSdnn   = obj.optDouble("hrv_sdnn"),
                hrvPnn50  = obj.optDouble("hrv_pnn50"),
                stress    = obj.optDouble("stress"),
                ax        = obj.optDouble("ax"),
                ay        = obj.optDouble("ay"),
                az        = obj.optDouble("az"),
                gx        = obj.optDouble("gx"),
                gy        = obj.optDouble("gy"),
                gz        = obj.optDouble("gz")
            )
            runOnUiThread { updateDashboard(reading) }
        } catch (e: Exception) {
            Log.e("JSON_PARSE", "Bad JSON: $jsonStr", e)
            runOnUiThread { showAlert("Invalid data received") }
        }
    }

    private fun updateDashboard(r: Reading) {
        binding.valueHr.text     = "${r.bpm.round1()} BPM"
        binding.valueSpo2.text   = "${r.spo2.round1()} %"
        binding.valueTemp.text   = "${r.temp.round2()} °C"
        binding.valueGsr.text    = r.gsr.round0().toString()
        binding.valueHrv.text    = "${r.hrvRmssd.round0()} ms"
        binding.valueStress.text = r.stress.round1().toString()

        val issues = mutableListOf<String>()
        if (r.bpm > Thresholds.HR_HIGH) issues += "High Heart Rate"
        if (r.spo2 < Thresholds.SPO2_LOW) issues += "Low SpO₂"
        if (r.temp > Thresholds.TEMP_HIGH) issues += "High Temperature"
        if (r.temp < Thresholds.TEMP_LOW) issues += "Low Temperature"
        if (r.stress > Thresholds.STRESS_HIGH) issues += "High Stress"
        if (r.hrvRmssd < Thresholds.HRV_RMSSD_LOW) issues += "Low HRV"

        if (issues.isNotEmpty()) {
            showAlert("⚠ " + issues.joinToString(" • "))
        } else {
            binding.alertBanner.visibility = View.GONE
        }

        // Optional: color highlights for cards (simple example)
        setCardState(binding.valueHr, r.bpm > Thresholds.HR_HIGH)
        setCardState(binding.valueSpo2, r.spo2 < Thresholds.SPO2_LOW)
        setCardState(binding.valueTemp, r.temp !in Thresholds.TEMP_LOW..Thresholds.TEMP_HIGH)
        setCardState(binding.valueStress, r.stress > Thresholds.STRESS_HIGH)
    }

    private fun setCardState(valueView: TextView, alert: Boolean) {
        valueView.setTextColor(
            if (alert) getColor(android.R.color.holo_red_dark)
            else getColor(R.color.charcoalText)
        )
    }

    private fun showAlert(message: String) {
        binding.alertBanner.text = message
        binding.alertBanner.visibility = View.VISIBLE
    }

    // --- Model & helpers ---
    data class Reading(
        val cycleId: Int,
        val timestamp: Long,
        val bpm: Double,
        val spo2: Double,
        val temp: Double,
        val gsr: Double,
        val hrvRmssd: Double,
        val hrvSdnn: Double,
        val hrvPnn50: Double,
        val stress: Double,
        val ax: Double, val ay: Double, val az: Double,
        val gx: Double, val gy: Double, val gz: Double
    )

    private fun Double.round1() = String.format(Locale.US, "%.1f", this)
    private fun Double.round2() = String.format(Locale.US, "%.2f", this)
    private fun Double.round0() = String.format(Locale.US, "%.0f", this).toInt()
}
