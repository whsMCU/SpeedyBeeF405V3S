package com.wang.mcu_drone

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.recyclerview.widget.LinearLayoutManager
import com.wang.mcu_drone.bluetooth.BleClientAdapter
import com.wang.mcu_drone.bluetooth.BleClientGattCallback
import com.wang.mcu_drone.databinding.ActivityMainBinding
import com.wang.mcu_drone.utils.checkConnectPermission
import com.wang.mcu_drone.utils.getDeviceName
import com.wang.mcu_drone.utils.toast
import java.util.UUID


val UUID_SERVICE = UUID.fromString("0000abf0-0000-1000-8000-00805f9b34fb")
val UUID_CHA_READ_NOTIFY = UUID.fromString("0000abf2-0000-1000-8000-00805f9b34fb")
val UUID_DESC_NOTIFY = UUID.fromString("11100000-0000-0000-0000-000000000000")
val UUID_CAHR_WRITE = UUID.fromString("0000abf1-0000-1000-8000-00805f9b34fb")


class MainActivity : AppCompatActivity() {

    private val binding by lazy { ActivityMainBinding.inflate(layoutInflater) }
    private var mBleAdapter: BluetoothAdapter? = null
    private val mListAdapter = BleClientAdapter()

    private var mGatt: BluetoothGatt? = null

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            super.onScanResult(callbackType, result)
            if (result == null) return
            if (getDeviceName(result.device).isNullOrEmpty()) return
            val notHas = mListAdapter.items.none { it.device.address == result.device.address}
            if (notHas) mListAdapter.add(result)
        }
    }
    private val gattCallback = BleClientGattCallback(this) { logPrint(it) }

    private lateinit var requestPermissions: ActivityResultLauncher<Array<String>>
    private lateinit var requestActivityResult: ActivityResultLauncher<Intent>

    private var haveAllCondition = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        setContentView(binding.root)

        initLaunch()
        checkPermission()
        initView()
        val bleManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        mBleAdapter = bleManager.adapter

        mBleAdapter?.let { checkConnectPermission { it.bluetoothLeScanner.startScan(scanCallback) } }
    }


    private fun initLaunch() {
        requestPermissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { result ->
            val isSuccess = result.none { it.key != android.Manifest.permission.READ_EXTERNAL_STORAGE && !it.value }
            if (isSuccess) {
                checkBle()
            } else {
                toast("블루투스 권한 없음...")
            }
        }
        requestActivityResult = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) {
            val bleManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
            val bleAdapter = bleManager.adapter
            val isEnable = bleAdapter.isEnabled
            if (isEnable) {
                haveAllCondition = true
            } else {
                haveAllCondition = false
                toast("블루투스가 켜지지 않았습니다...")
            }
        }
    }

    private fun initView() {
        binding.toolbar.setTitle(R.string.app_name)
        mListAdapter.setOnItemClickListener { a, _, p ->
            a.getItem(p)?.device?.let { connect(it) }
        }
        binding.recyclerView.apply {
            layoutManager = LinearLayoutManager(this@MainActivity)
            adapter = mListAdapter
        }
        binding.scan.setOnClickListener { startScan() }
        binding.setNotify.setOnClickListener { mGatt?.let { setNotify(it) } }
        binding.read.setOnClickListener { mGatt?.let { read(it) } }
        binding.write.setOnClickListener { mGatt?.let { write(it) } }

        binding.buttonRead.setOnClickListener {  }
        binding.buttonWrite.setOnClickListener {  }
        binding.buttonSave.setOnClickListener {  }
    }

    private fun checkPermission() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            listOf(
                android.Manifest.permission.BLUETOOTH_CONNECT,
                android.Manifest.permission.BLUETOOTH_SCAN,
                android.Manifest.permission.ACCESS_FINE_LOCATION,
                android.Manifest.permission.READ_EXTERNAL_STORAGE
            )
        } else {
            listOf(
                android.Manifest.permission.BLUETOOTH,
                android.Manifest.permission.ACCESS_FINE_LOCATION,
                android.Manifest.permission.READ_EXTERNAL_STORAGE
            )
        }
        val notGrantedList = arrayListOf<String>()
        permissions.forEach {
            val isGranted = ActivityCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
            if (!isGranted) notGrantedList.add(it)
        }
        requestPermissions.launch(notGrantedList.toTypedArray())
    }

    private fun checkBle() {
        val bleManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        val bleAdapter = bleManager.adapter
        val isEnable = bleAdapter.isEnabled
        if (!isEnable) {
            requestActivityResult.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
        } else {
            haveAllCondition = true
        }
    }


    private fun startScan() {
        mBleAdapter?: toast("사용할 수 없음, 검사 권한과 블루투스...")
        mBleAdapter?.let {
            checkConnectPermission {
                mGatt?.disconnect()
                mGatt?.close()
                val scanner = it.bluetoothLeScanner
                mListAdapter.submitList(emptyList())
                scanner.stopScan(scanCallback)
                scanner.startScan(scanCallback)
            }
        }
    }

    private fun connect(device: BluetoothDevice) {
        checkConnectPermission {
            closeConnect()
            val address = device.address
            logPrint("[${address}]와 연결시작......")
            mGatt = device.connectGatt(this, false, gattCallback)
        }
    }

    private fun setNotify(gatt: BluetoothGatt) {
        val service = gatt.getService(UUID_SERVICE) ?: return
        val characteristic = service.getCharacteristic(UUID_CHA_READ_NOTIFY)
        checkConnectPermission {
            /// 设置Characteristic通知
            val setNotifyResult = gatt.setCharacteristicNotification(characteristic, true)
            val descriptor = characteristic.getDescriptor(UUID_DESC_NOTIFY) //UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
            // 服务端不主动发数据, 只通知客户端去读取数据
            // descriptor.value = BluetoothGattDescriptor.ENABLE_INDICATION_VALUE
            // 向Characteristic的Descriptor属性写入通知开关, 使蓝牙设备主动向手机发送数据
            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            val writeDescriptorResult = gatt.writeDescriptor(descriptor)
            val log = "[${gatt.device.address}]와 피처 알림 설정: $setNotifyResult, 설명하다: $writeDescriptorResult"
            logPrint(log)
        }
    }

    private fun read(gatt: BluetoothGatt) {
        val service = gatt.getService(UUID_SERVICE) ?: return
        val characteristic = service.getCharacteristic(UUID_CHA_READ_NOTIFY)
        checkConnectPermission { gatt.readCharacteristic(characteristic) }
    }

    private fun write(gatt: BluetoothGatt) {
        val text = binding.input.text.toString()
        logPrint("데이터 전송 : [${text}]")
        if (text.toByteArray().size > 20) {
            toast("최대 20바이트까지만 보낼 수 있습니다.")
            return
        }
        val service = gatt.getService(UUID_SERVICE) ?: return
        val characteristic = service.getCharacteristic(UUID_CAHR_WRITE)
        characteristic.setValue(text)
        checkConnectPermission { gatt.writeCharacteristic(characteristic) }
    }

    private fun closeConnect() {
        mGatt?.let {
            checkConnectPermission {
                it.disconnect()
                it.close()
            }
        }
    }

    private fun logPrint(text: String) {
        println(text)
        val tvText = binding.log.text.toString() + "\n" + text + "\n"
        Handler(Looper.getMainLooper()).post {
            binding.log.text = tvText
            val scrollY = binding.log.measuredHeight - binding.logLayout.height
            binding.logLayout.smoothScrollTo(0, scrollY)
        }
    }
}