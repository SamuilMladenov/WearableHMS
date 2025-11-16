package com.example.wearablehmsmobileapp

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View
import android.view.WindowManager
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.example.wearablehmsmobileapp.databinding.ActivityMainBinding
import org.json.JSONObject
import java.util.UUID

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    // BLE
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothLeScanner: BluetoothLeScanner? = null
    private var bluetoothGatt: BluetoothGatt? = null

    // JSON buffer (for chunked BLE packets)
    private val buffer = StringBuilder()

    // Your Nordic UART UUIDs
    private val TARGET_DEVICE_NAME = "WearableHMS"
    private val SERVICE_UUID =
        UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    private val CHARACTERISTIC_UUID =
        UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")

    // Thresholds
    object Thresholds {
        const val HR_HIGH = 100.0
        const val SPO2_LOW = 92.0
        const val TEMP_HIGH = 38.0
        const val STRESS_HIGH = 70.0
        const val HRV_LOW = 30.0
    }

    // BLUETOOTH PERMISSIONS
    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { perms ->
        if (perms.values.all { it }) startBleScan()
        else showAlert("Bluetooth permissions denied")
    }

    // ---------------------------------------------------------
    // onCreate
    // ---------------------------------------------------------
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner

        requestBlePermissions()
    }

    // ---------------------------------------------------------
    // BLE PERMISSIONS
    // ---------------------------------------------------------
    private fun requestBlePermissions() {
        val needed = mutableListOf<String>()

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            needed += Manifest.permission.BLUETOOTH_SCAN
            needed += Manifest.permission.BLUETOOTH_CONNECT
        } else {
            needed += Manifest.permission.ACCESS_FINE_LOCATION
        }

        permissionLauncher.launch(needed.toTypedArray())
    }

    // ---------------------------------------------------------
    // BLE SCANNING
    // ---------------------------------------------------------
    @SuppressLint("MissingPermission")
    private fun startBleScan() {
        showBanner("Scanning for $TARGET_DEVICE_NAME…")

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        bluetoothLeScanner?.startScan(null, settings, scanCallback)
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(type: Int, result: ScanResult) {
            val name = result.device.name ?: return

            Log.d("BLE_SCAN", "Found: $name")

            if (name == TARGET_DEVICE_NAME) {
                showBanner("Connecting to $name…")
                bluetoothLeScanner?.stopScan(this)

                result.device.connectGatt(
                    this@MainActivity,
                    false,
                    gattCallback
                )
            }
        }
    }

    // ---------------------------------------------------------
    // GATT CALLBACKS
    // ---------------------------------------------------------
    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(
            gatt: BluetoothGatt,
            status: Int,
            newState: Int
        ) {
            Log.d("BLE", "onConnectionStateChange: status=$status newState=$newState")

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                bluetoothGatt = gatt
                showBanner("Connected • Discovering services…")
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                showAlert("Disconnected")
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            val service = gatt.getService(SERVICE_UUID)
            if (service == null) {
                showAlert("Service not found")
                return
            }

            val characteristic = service.getCharacteristic(CHARACTERISTIC_UUID)
            if (characteristic == null) {
                showAlert("Characteristic not found")
                return
            }

            Log.d("BLE", "Found notify characteristic")

            runOnUiThread {
                binding.alertBanner.text = "Connected • Waiting for measurement (~90s)…"
                binding.alertBanner.visibility = View.VISIBLE
            }

            gatt.setCharacteristicNotification(characteristic, true)

            val cccd = characteristic.getDescriptor(
                UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
            )
            cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            gatt.writeDescriptor(cccd)
        }

        override fun onDescriptorWrite(
            gatt: BluetoothGatt,
            descriptor: BluetoothGattDescriptor,
            status: Int
        ) {
            Log.d("BLE_CCCD", "Descriptor write completed. Status = $status")
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            val raw = characteristic.value ?: return

            Log.d("BLE_RAW", raw.joinToString(",") { it.toString() })

            val text = raw.toString(Charsets.UTF_8)
            Log.d("BLE_JSON", "Chunk: \"$text\"")
            Log.d("BLE_NOTIFY", "onCharacteristicChanged called")

            buffer.append(text)

            // Try to extract as many complete JSON objects as possible
            while (true) {
                // Look for a closing brace
                val endIndex = buffer.indexOf("}")
                if (endIndex == -1) {
                    // No full JSON yet
                    break
                }

                val fullJson = buffer.substring(0, endIndex + 1)
                buffer.delete(0, endIndex + 1)

                Log.d("BLE_JSON", "FULL JSON = $fullJson")

                try {
                    val obj = JSONObject(fullJson.trim())
                    // Mark that we actually got data
                    runOnUiThread {
                        binding.alertBanner.text = "Measurement received"
                        binding.alertBanner.visibility = View.VISIBLE
                    }
                    updateUI(obj)
                } catch (e: Exception) {
                    Log.e("BLE_JSON", "Invalid JSON: $fullJson", e)
                }
            }
        }
    }

    // ---------------------------------------------------------
    // UPDATE UI
    // ---------------------------------------------------------
    private fun updateUI(json: JSONObject) {

        val bpm = json.optDouble("bpm")
        val spo2 = json.optDouble("spo2")
        val temp = json.optDouble("temp")
        val gsr = json.optDouble("gsr")
        val stress = json.optDouble("stress")
        val hrv = json.optDouble("hrv_rmssd")

        // Update text
        runOnUiThread {
            binding.valueHr.text = "${bpm.toInt()} BPM"
            binding.valueSpo2.text = "${spo2.toInt()} %"
            binding.valueTemp.text = String.format("%.2f °C", temp)
            binding.valueGsr.text = gsr.toInt().toString()
            binding.valueHrv.text = "${hrv.toInt()} ms"
            binding.valueStress.text = stress.toInt().toString()
        }

        // Alerts
        val alerts = mutableListOf<String>()

        if (bpm > Thresholds.HR_HIGH) alerts += "High Heart Rate"
        if (spo2 < Thresholds.SPO2_LOW) alerts += "Low SpO₂"
        if (temp > Thresholds.TEMP_HIGH) alerts += "High Temperature"
        if (stress > Thresholds.STRESS_HIGH) alerts += "High Stress"
        if (hrv < Thresholds.HRV_LOW) alerts += "Low HRV"

        if (alerts.isNotEmpty()) showAlert(alerts.joinToString(" • "))
    }

    // ---------------------------------------------------------
    // ALERTS & BANNERS
    // ---------------------------------------------------------
    private fun showBanner(text: String) {
        runOnUiThread {
            binding.alertBanner.text = text
            binding.alertBanner.visibility = View.VISIBLE
        }
    }

    private fun showAlert(text: String) {
        runOnUiThread {
            binding.alertBanner.text = text
            binding.alertBanner.visibility = View.VISIBLE

            binding.alertBanner.postDelayed({
                binding.alertBanner.visibility = View.GONE
            }, 3000)
        }
    }
}
