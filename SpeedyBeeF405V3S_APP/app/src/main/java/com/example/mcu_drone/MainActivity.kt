package com.example.mcu_drone

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import com.example.mcu_drone.ble.BleClientActivity
import com.example.mcu_drone.ble.BleServerActivity
import com.example.mcu_drone.bt.BtClientActivity
import com.example.mcu_drone.bt.BtServerActivity
import com.example.mcu_drone.databinding.ActivityMainBinding
import com.example.mcu_drone.utils.toast

const val INTENT_STATUS = "status"

class MainActivity : AppCompatActivity() {

    private val binding by lazy { ActivityMainBinding.inflate(layoutInflater) }

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
    }


    private fun initLaunch() {
        requestPermissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { result ->
            val isSuccess = result.none { it.key != android.Manifest.permission.READ_EXTERNAL_STORAGE && !it.value }
            if (isSuccess) {
                checkBle()
            } else {
                toast("无蓝牙权限...")
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
                toast("蓝牙未开启...")
            }
        }
    }

    private fun initView() {
        binding.toolbar.setTitle(R.string.app_name)
        binding.bleClient.setOnClickListener {
            val intent = Intent(this, BleClientActivity::class.java)
            intent.putExtra(INTENT_STATUS, haveAllCondition)
            startActivity(intent)
        }
        binding.bleServer.setOnClickListener {
            val intent = Intent(this, BleServerActivity::class.java)
            intent.putExtra(INTENT_STATUS, haveAllCondition)
            startActivity(intent)
        }
        binding.btClient.setOnClickListener {
            val intent = Intent(this, BtClientActivity::class.java)
            intent.putExtra(INTENT_STATUS, haveAllCondition)
            startActivity(intent)
        }
        binding.bteServer.setOnClickListener {
            val intent = Intent(this, BtServerActivity::class.java)
            intent.putExtra(INTENT_STATUS, haveAllCondition)
            startActivity(intent)
        }
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
}