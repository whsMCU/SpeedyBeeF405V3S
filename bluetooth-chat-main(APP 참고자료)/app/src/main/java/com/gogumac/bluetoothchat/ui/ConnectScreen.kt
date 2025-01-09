package com.gogumac.bluetoothchat.ui

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material3.Text
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.MailOutline
import androidx.compose.material3.Button
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.Surface
import androidx.compose.material3.SwipeToDismissBox
import androidx.compose.material3.SwipeToDismissBoxValue
import androidx.compose.material3.TextButton
import androidx.compose.material3.rememberSwipeToDismissBoxState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gogumac.bluetoothchat.R
import com.gogumac.bluetoothchat.ui.theme.BluetoothChatTheme
import com.gogumac.bluetoothchat.ui.theme.ConnectBackground


@SuppressLint("MissingPermission")
@Composable
fun ConnectScreen(
    modifier: Modifier = Modifier,
    deviceList: List<BluetoothDevice>,
    onBluetoothDeviceScanRequest: () -> Unit,
    onDeviceConnectRequest: (String) -> Unit,
    onSetDiscoverableRequest: () -> Unit,
    onServerSocketOpenRequested: () -> Unit
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp)
    ) {
        Column(modifier = modifier.weight(1f)) {
            Text("저장된 기기 목록")
            HorizontalDivider(modifier = Modifier.padding(vertical = 8.dp))
            LazyColumn(verticalArrangement = Arrangement.spacedBy(4.dp)) {
                items(deviceList) {
                    val name = it.name
                    val address = it.address
                    SwipeDeviceItem(
                        name = name,
                        macAddress = address,
                        requestConnectDevice = { onDeviceConnectRequest(address) })
                }
                item {
                    TextButton(onClick = { onBluetoothDeviceScanRequest() }) {
                        Text(text = "+ 새 기기 연결하기", modifier = Modifier)
                    }
                }
            }
        }
        Row(modifier = Modifier.height(64.dp)) {
            Button(
                onClick = onSetDiscoverableRequest,
                modifier = Modifier
                    .weight(1f)
                    .fillMaxHeight()
            ) {
                Text(
                    text = stringResource(id = R.string.set_discoverable),
                    style = TextStyle(fontSize = 14.sp),
                    fontWeight = FontWeight.Bold
                )
            }
            Spacer(modifier = Modifier.size(8.dp))
            Button(
                onClick = onServerSocketOpenRequested,
                modifier = Modifier
                    .weight(1f)
                    .fillMaxHeight()
            ) {
                Text(
                    text = stringResource(id = R.string.open_server_socket),
                    style = TextStyle(fontSize = 14.sp),
                    fontWeight = FontWeight.Bold
                )
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SwipeDeviceItem(
    modifier: Modifier = Modifier,
    name: String?,
    macAddress: String,
    requestConnectDevice: () -> Unit
) {
    val swipeState =
        rememberSwipeToDismissBoxState(positionalThreshold = { it * 0.2f }, confirmValueChange = {

            if (it == SwipeToDismissBoxValue.EndToStart) {
                requestConnectDevice()
            }
            false
        })
    val scale by animateFloatAsState(
        targetValue = if (swipeState.targetValue == SwipeToDismissBoxValue.Settled) 1f else 1.25f,
        label = ""
    )
    SwipeToDismissBox(
        modifier = modifier,
        state = swipeState,
        enableDismissFromStartToEnd = false,
        backgroundContent = {
            Surface(
                modifier = Modifier.fillMaxSize(),
                shape = RoundedCornerShape(8.dp),
                color = ConnectBackground
            ) {

                Box(
                    contentAlignment = Alignment.CenterEnd, modifier = Modifier
                        .fillMaxSize()
                        .padding(horizontal = 8.dp)
                ) {
                    Icon(
                        imageVector = Icons.Default.MailOutline,
                        contentDescription = null,
                        tint = Color.White,
                        modifier = Modifier.scale(scale)
                    )
                }
            }
        }) {
        BluetoothDeviceItem(name = name, macAddress = macAddress)
    }
}

@Composable
fun BluetoothDeviceItem(
    modifier: Modifier = Modifier,
    name: String?,
    macAddress: String,
    borderColor: Color = Color.LightGray,
    borderWidth: Dp = 0.4.dp
) {
    Surface(
        modifier = modifier
            .height(64.dp)
            .fillMaxWidth()
            .border(width = borderWidth, color = borderColor, shape = RoundedCornerShape(8.dp)),
        shape = RoundedCornerShape(8.dp),
        color = Color.White
    ) {
        Column(
            modifier = Modifier.padding(12.dp),
            verticalArrangement = Arrangement.SpaceBetween
        ) {
            Text(
                text = name ?: "no name",
                modifier = Modifier,
                overflow = TextOverflow.Ellipsis,
                maxLines = 1
            )
            Text(
                text = "mac : $macAddress",
                style = TextStyle(color = Color.Gray, fontSize = 12.sp),
                modifier = Modifier
            )
        }
    }
}


@Preview(showBackground = true)
@Composable
fun ConnectScreenPreview() {
    BluetoothChatTheme {
        ConnectScreen(
            deviceList = listOf(),
            onBluetoothDeviceScanRequest = {},
            onDeviceConnectRequest = {},
            onServerSocketOpenRequested = {},
            onSetDiscoverableRequest = {})
    }
}

@Preview(showBackground = true)
@Composable
fun DeviceItemPreview() {
    BluetoothChatTheme {
        BluetoothDeviceItem(
            name = "TEST1".repeat(10),
            macAddress = "12:34:56:78:90"
        )
    }
}
