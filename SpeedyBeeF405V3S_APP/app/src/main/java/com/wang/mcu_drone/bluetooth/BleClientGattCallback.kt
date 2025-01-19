package com.wang.mcu_drone.bluetooth

import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothProfile
import android.content.Context
import com.wang.mcu_drone.utils.checkConnectPermission


/**
 * FileName: BleGattCallback
 * Author: JiaoCan
 * Date: 2024/5/7 16:15
 */

class BleClientGattCallback(context: Context, func: (log: String) -> Unit, parser: (data: ByteArray) -> Unit) : BluetoothGattCallback() {

    private val mContext = context
    private val mFunc = func
    private val mparser = parser

    override fun onServicesDiscovered(gatt: BluetoothGatt?, status: Int) {
        super.onServicesDiscovered(gatt, status)
        if (gatt == null) return
        gatt.services.forEach { s ->
            val log = StringBuilder().appendLine("[${gatt.device.address}]디스커버리 서비스:")
                .appendLine("S=${s.uuid}")
            s.characteristics.forEach { c ->
                log.appendLine("C=${c.uuid}")
                c.descriptors.forEach { d ->
                    log.appendLine("D=${d.uuid}")
                }
            }
            mFunc.invoke(log.toString())
        }
    }

    override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
        super.onConnectionStateChange(gatt, status, newState)
        if (gatt == null) return
        val address = gatt.device.address
        val isConnected = BluetoothGatt.GATT_SUCCESS == status && BluetoothProfile.STATE_CONNECTED == newState
        if (isConnected) {
            mContext.checkConnectPermission { gatt.discoverServices() }
        } else {
            mContext.checkConnectPermission {
                gatt.disconnect()
                gatt.close()
            }
        }
        val log = "[${address}]와" + when {
            isConnected -> " 연결성공"
            newState == BluetoothProfile.STATE_DISCONNECTED -> {
                if (status == 0) "능동적으로 연결 끊기" else "자동으로 연결 끊기"
            }
            else -> "연결오류, 오류코드:$status"
        }
        mFunc.invoke(log)
    }

    override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
        super.onCharacteristicChanged(gatt, characteristic, value)
        val charValue = characteristic.value
        val log = "[${gatt.device.address}]데이터 읽기(10진수)\n${charValue.joinToString(", ")}\nUTF-8:${String(charValue)}"
        mFunc.invoke(log)
        mparser.invoke(charValue)
    }

    override fun onCharacteristicRead(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray, status: Int) {
        super.onCharacteristicRead(gatt, characteristic, value, status)
        val charValue = characteristic.value
        val log = "[${gatt.device.address}]데이터 읽기(10진수)\n${charValue.joinToString(", ")}\nUTF-8:${String(charValue)}"
        mFunc.invoke(log)
    }

    override fun onCharacteristicWrite(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?, status: Int) {
        super.onCharacteristicWrite(gatt, characteristic, status)
        if (gatt == null || characteristic == null) return
        val charValue = characteristic.value
        val log = "[${gatt.device.address}]데이터 쓰기(10진수)\n${charValue.joinToString(", ")}\nUTF-8:${String(charValue)}"
        mFunc.invoke(log)
    }


}

