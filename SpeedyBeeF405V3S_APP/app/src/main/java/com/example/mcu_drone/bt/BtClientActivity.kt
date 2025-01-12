package com.example.mcu_drone.bt

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.Bundle
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.mcu_drone.INTENT_STATUS
import com.example.mcu_drone.R
import com.example.mcu_drone.databinding.ActivityBtClientBinding
import com.example.mcu_drone.utils.FileUtil
import com.example.mcu_drone.utils.checkConnectPermission
import com.example.mcu_drone.utils.getOrNull
import com.example.mcu_drone.utils.toast
import java.text.SimpleDateFormat

class BtClientActivity : AppCompatActivity(), BtBase.BtListener {

    private val binding by lazy { ActivityBtClientBinding.inflate(layoutInflater) }
    private var mBleAdapter: BluetoothAdapter? = null
    private val mListAdapter = BtClientAdapter()
    private val mClient: BtClient = BtClient(this).apply {
        setBtListener(this@BtClientActivity)
    }
    private var curLog = ""
    @SuppressLint("SimpleDateFormat")
    private val dataFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss")
    private lateinit var requestFile: ActivityResultLauncher<String>

    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent == null || intent.action != BluetoothDevice.ACTION_FOUND) return
            val device: BluetoothDevice? = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
            if (device == null) return
            val notHas = mListAdapter.items.none { it.address == device.address}
            if (notHas) mListAdapter.add(device)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(binding.root)

        val status = intent.getBooleanExtra(INTENT_STATUS, false)
        if (status) {
            val bleManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
            mBleAdapter = bleManager.adapter
        }
        initLaunch()
        initView()
        registerReceiver(receiver, IntentFilter(BluetoothDevice.ACTION_FOUND))
        mBleAdapter?.let { startScan() }
    }

    override fun onDestroy() {
        super.onDestroy()
        mClient.setBtListener(null)
        mClient.closeConnect()
        unregisterReceiver(receiver)
        checkConnectPermission {
            mBleAdapter?.cancelDiscovery()
        }
    }

    private fun initLaunch() {
        requestFile = registerForActivityResult(ActivityResultContracts.GetContent()) {
            if (it == null) {
                toast("선택한 파일이 없습니다...")
                return@registerForActivityResult
            }
            getOrNull { FileUtil.uriToPath(this, it) }?.let { path ->
                println("시험: $path")
                mClient.sendClientFile(path)
            }
        }
    }


    private fun initView() {
        binding.toolbar.setTitle(R.string.bt_client)
        mListAdapter.setOnItemClickListener { a, _, p ->
            a.getItem(p)?.let { connect(it) }
        }
        binding.recyclerView.apply {
            layoutManager = LinearLayoutManager(this@BtClientActivity)
            adapter = mListAdapter
        }
        binding.scan.setOnClickListener { startScan() }
        binding.sendText.setOnClickListener { sendText() }
        binding.sendFile.setOnClickListener { sendFile() }
        binding.disconnect.setOnClickListener { disconnect() }
    }

    private fun startScan() {
        mBleAdapter?: toast("이용불가, 권한 및 블루투스 확인...")
        mBleAdapter?.let {
            checkConnectPermission {
                mListAdapter.submitList(emptyList())
                mListAdapter.addAll(it.bondedDevices.toList())
                it.cancelDiscovery()
                it.startDiscovery()
            }
        }
    }

    private fun connect(device: BluetoothDevice) {
        if (mClient.isConnectedWithDevice(device)) {
            toast("장치가 연결되었습니다.")
            return
        }
        mClient.connect(device)
    }

    private fun disconnect() {
        if (!mClient.isConnected()) {
            toast("연결되지 않음...")
            return
        }
        mClient.closeConnect()
    }

    private fun sendText() {
        if (!mClient.isConnected()) {
            toast("연결되지 않음...")
            return
        }
        val text = binding.input.text.toString()
        if (text.isEmpty()) {
            toast("보낼 데이터 없음...")
            return
        }
        mClient.sendClientMsg(text)
    }

    private fun sendFile() {
        if (!mClient.isConnected()) {
            toast("연결되지 않음...")
            return
        }
        requestFile.launch("*/*")
    }

    override fun onBtStateChanged(address: String, state: BtBase.BtState) {
        when (state) {
            BtBase.BtState.CONNECTING -> logPrint("[$address]와 연결중...")
            BtBase.BtState.CONNECTED -> logPrint("[$address]와 연결성공")
            BtBase.BtState.DISCONNECT -> logPrint("[$address]와 연결끊기")
            BtBase.BtState.BUSY -> logPrint("다른데이터 보네기, 나중에 다시 보내주세요...")
            BtBase.BtState.ERROR -> logPrint("[$address]와 연결오류")
        }
    }

    override fun onMsgOperated(address: String, msg: String, isSender: Boolean) {
        if (isSender) {
            logPrint("[$address]로 메세지 보내기: $msg")
        } else {
            logPrint("[$address]에서 메세지 수신됨: $msg")
        }
    }

    override fun onFileOperated(address: String, fileName: String, progress: Int, isSender: Boolean, savePath: String?) {
        if (progress == 0) {
            if (isSender) {
                logPrint("[$address]로 파일 보내기($fileName)中...")
            } else {
                logPrint("[$address]에서 파일 수신($fileName)中...")
            }
        } else if (progress == 100) {
            if (isSender) {
                logPrint("[$address]로 파일 보내기($fileName)완료")
            } else {
                logPrint("[$address]에서 파일 수신($fileName)완료\n저장경로:$savePath")
            }
        }
    }

    override fun onOperateErrorLog(log: String) {
        logPrint(log)
    }

    private fun logPrint(text: String) {
        println(text)
        curLog = curLog + "\n" + dataFormat.format(System.currentTimeMillis()) + "\n" + text + "\n"
        binding.log.text = curLog
        val scrollY = binding.log.measuredHeight - binding.logLayout.height
        binding.logLayout.smoothScrollTo(0, scrollY)
    }
}