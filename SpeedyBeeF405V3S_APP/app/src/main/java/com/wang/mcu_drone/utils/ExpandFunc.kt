package com.wang.mcu_drone.utils

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.content.Context
import android.content.pm.PackageManager
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Toast
import androidx.core.app.ActivityCompat
import com.wang.mcu_drone.MyApplication


/**
 * FileName: ExpandFunc
 * Author: JiaoCan
 * Date: 2024/5/7
 */


fun toast(text: String) {
    MyApplication.getContext().apply {
        Handler(Looper.getMainLooper()).post {
            Toast.makeText(this, text, Toast.LENGTH_SHORT).show()
        }
    }
}

fun Context.checkConnectPermission(func: () -> Unit) {
    if (ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED
    ) {
        func.invoke()
    } else {
        Log.e("SDKTEST", "테스트: 연결권한없음")
    }
}

fun Context.checkAdvertisePermission(func: () -> Unit) {
    if (ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.BLUETOOTH_ADVERTISE
        ) == PackageManager.PERMISSION_GRANTED
    ) {
        func.invoke()
    } else {
        Log.e("SDKTEST", "테스트: 방송권한없음")
    }
}

fun Context.getDeviceName(device: BluetoothDevice): String? {
    return if (ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED
    ) {
        device.name
    } else {
        null
    }
}

fun Context.getDeviceBond(device: BluetoothDevice): Int? {
    return if (ActivityCompat.checkSelfPermission(
            this,
            Manifest.permission.BLUETOOTH_CONNECT
        ) == PackageManager.PERMISSION_GRANTED
    ) {
        device.bondState
    } else {
        null
    }
}

fun <T> getOrNull(func: () -> T): T? {
    return runCatching { func.invoke() }.getOrNull()
}

