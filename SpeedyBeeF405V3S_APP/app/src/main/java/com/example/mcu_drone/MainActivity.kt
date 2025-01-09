package com.example.mcu_drone

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.os.Bundle
import android.os.Handler
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import java.util.UUID


class MainActivity : AppCompatActivity() {

    var mTvBluetoothStatus: TextView? = null
    var mTvReceiveData: TextView? = findViewById(R.id.tvReceiveData)
    var mTvSendData: TextView? = findViewById(R.id.tvSendData)
    var mBtnBluetoothOn: Button? = findViewById(R.id.btnBluetoothOn)
    var mBtnBluetoothOff: Button? = findViewById(R.id.btnBluetoothOff)
    var mBtnConnect: Button? = findViewById(R.id.btnConnect)
    var mBtnSendData: Button? = findViewById(R.id.btnSendData)

    var mBluetoothAdapter: BluetoothAdapter? = null
    var mPairedDevices: Set<BluetoothDevice>? = null
    var mListPairedDevices: List<String>? = null

    var mBluetoothHandler: Handler? = null
//    var mThreadConnectedBluetooth: ConnectedBluetoothThread? = null
    var mBluetoothDevice: BluetoothDevice? = null
    var mBluetoothSocket: BluetoothSocket? = null

    val BT_REQUEST_ENABLE: Int = 1
    val BT_MESSAGE_READ: Int = 2
    val BT_CONNECTING_STATUS: Int = 3
    val BT_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        //enableEdgeToEdge()
        setContentView(R.layout.activity_main)

        mTvBluetoothStatus = findViewById(R.id.tvBluetoothStatus)
        mTvReceiveData = findViewById(R.id.tvReceiveData)
        mTvSendData = findViewById(R.id.tvSendData)
        mBtnBluetoothOn = findViewById(R.id.btnBluetoothOn)
        mBtnBluetoothOff = findViewById(R.id.btnBluetoothOff)
        mBtnConnect = findViewById(R.id.btnConnect)
        mBtnSendData = findViewById(R.id.btnSendData)

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

        mBtnBluetoothOn.setOnClickListener {
            bluetoothOn()
        }

        mBtnBluetoothOff.setOnClickListener {
            bluetoothOff()
        }

        mBtnConnect.setOnClickListener {
            listPairedDevices()
        }

        mBtnSendData.setOnClickListener {
            if(mThreadConnectedBluetooth != null){
                mThreadConnectedBluetooth.write(mTvSendData.getText().toString())
            }
        }

        https://bugwhale.tistory.com/entry/android-bluetooth-application
    }


}