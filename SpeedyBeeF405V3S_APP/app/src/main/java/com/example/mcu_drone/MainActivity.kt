package com.example.mcu_drone

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Handler
import android.util.Log
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.ListView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID


class MainActivity : AppCompatActivity() {

    private val bluetoothAdapter: BluetoothAdapter? by lazy { BluetoothAdapter.getDefaultAdapter() }
    private lateinit var devicesAdapter: ArrayAdapter<String>
    private val devicesList = mutableListOf<BluetoothDevice>()
    private val uuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

    private var bluetoothSocket: BluetoothSocket? = null
    private var inputStream: InputStream? = null
    private var outputStream: OutputStream? = null

    private lateinit var messageInput: EditText
    private lateinit var sendButton: Button
    private lateinit var receivedMessages: TextView

    private val handler = Handler()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        //enableEdgeToEdge()
        setContentView(R.layout.activity_main)
        val scanButton: Button = findViewById(R.id.scanButton)
        val devicesListView: ListView = findViewById(R.id.devicesListView)
        messageInput = findViewById(R.id.messageInput)
        sendButton = findViewById(R.id.sendButton)
        receivedMessages = findViewById(R.id.receivedMessages)

        devicesAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1)
        devicesListView.adapter = devicesAdapter

        scanButton.setOnClickListener {
            checkPermissionsAndStartScan()
        }

        devicesListView.setOnItemClickListener { _, _, position, _ ->
            val device = devicesList[position]
            connectToDevice(device)
        }

        sendButton.setOnClickListener {
            val message = messageInput.text.toString()
            sendMessage(message)
        }

        val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        registerReceiver(receiver, filter)
    }

    override fun onDestroy() {
        super.onDestroy()
        unregisterReceiver(receiver)
        closeConnection()
    }

    private fun checkPermissionsAndStartScan() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED ||
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED
        ) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT,
                    Manifest.permission.ACCESS_FINE_LOCATION
                ),
                REQUEST_BLUETOOTH_PERMISSIONS
            )
        } else {
            startBluetoothScan()
        }
    }

    private fun startBluetoothScan() {
        bluetoothAdapter?.startDiscovery()
        Toast.makeText(this, "블루투스 스캔 시작", Toast.LENGTH_SHORT).show()
    }

    private fun connectToDevice(device: BluetoothDevice) {
        bluetoothAdapter?.cancelDiscovery()
        try {
            bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
            bluetoothSocket?.connect()
            inputStream = bluetoothSocket?.inputStream
            outputStream = bluetoothSocket?.outputStream

            Toast.makeText(this, "디바이스에 연결 성공: ${device.name}", Toast.LENGTH_SHORT).show()
            startListeningForData()
        } catch (e: IOException) {
            Log.e("Bluetooth", "연결 실패: ${e.message}")
            Toast.makeText(this, "디바이스에 연결 실패", Toast.LENGTH_SHORT).show()
        }
    }

    private fun sendMessage(message: String) {
        try {
            outputStream?.write(message.toByteArray())
            Toast.makeText(this, "메시지 전송: $message", Toast.LENGTH_SHORT).show()
        } catch (e: IOException) {
            Log.e("Bluetooth", "메시지 전송 실패: ${e.message}")
            Toast.makeText(this, "메시지 전송 실패", Toast.LENGTH_SHORT).show()
        }
    }

    private fun startListeningForData() {
        Thread {
            val buffer = ByteArray(1024)
            var bytes: Int

            while (true) {
                try {
                    bytes = inputStream?.read(buffer) ?: break
                    val message = String(buffer, 0, bytes)
                    handler.post {
                        receivedMessages.append("수신: $message\n")
                    }
                } catch (e: IOException) {
                    Log.e("Bluetooth", "데이터 수신 실패: ${e.message}")
                    break
                }
            }
        }.start()
    }

    private fun closeConnection() {
        try {
            inputStream?.close()
            outputStream?.close()
            bluetoothSocket?.close()
        } catch (e: IOException) {
            Log.e("Bluetooth", "소켓 종료 실패: ${e.message}")
        }
    }

    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            when (intent.action) {
                BluetoothDevice.ACTION_FOUND -> {
                    val device: BluetoothDevice? =
                        intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                    device?.let {
                        if (!devicesList.contains(it)) {
                            devicesList.add(it)
                            devicesAdapter.add(it.name ?: it.address)
                            devicesAdapter.notifyDataSetChanged()
                        }
                    }
                }
            }
        }
    }

    companion object {
        private const val REQUEST_BLUETOOTH_PERMISSIONS = 1
    }


}