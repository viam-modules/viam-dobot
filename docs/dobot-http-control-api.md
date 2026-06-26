# Dobot CR controller — HTTP/JSON control API (reference)

> **Status: reverse-engineered, unofficial.** This document was reconstructed by
> decompiling **DobotStudioPro for Android, v4.6.1.0** (`DobotStudioPro-App-4.6.1.0-stable-202502261200.apk`)
> with `jadx`, and reading its `Retrofit2` service interfaces under
> `cc.dobot.dobotstudio.http`. It describes the **private HTTP API the DobotStudioPro
> GUI uses to drive the controller** — the same backend the controller's web UI talks to.
>
> It is **not** the public, vendor-documented integration interface. For third-party
> drivers (including this Viam module), the supported channel is the **TCP/IP Remote
> Control ASCII protocol** (dashboard port `29999`, realtime feedback port `30004`)
> described in *Dobot TCP_IP Remote Control Interface Guide V4.x*. This HTTP API is
> documented here only as a **reference** — e.g. to see how the GUI sets robot mode —
> and it may change between firmware/app versions without notice. Prefer the ASCII
> protocol unless you specifically need something only exposed here.

## Transport

| Property | Value |
|---|---|
| Protocol | HTTP/1.1, JSON request/response |
| Client stack | Retrofit2 + OkHttp3 + Gson (`GsonConverterFactory`) |
| Base URL | `http://<controllerIP>:<port>/` — built in `RetrofitConfig.java:31` as `"http://" + AppData.getIP() + ":" + AppData.getPort() + "/"` |
| Control port | **22000** (default seen in the binary; the port is configurable in the app via `AppData.getPort()`). A secondary port `22001` is also referenced. The "Dobot+" plugin API uses a separate `AppData.getDobotPlusPort()`. |
| Request `Content-Type` | `application/json; charset=UTF-8` for model bodies; some endpoints use `text/plain` (`RetrofitConfig.createRequestBody`) |
| Timeouts | connect 3 s, read/write 60 s (`RetrofitConfig.java:30`) |
| Auth | none at the transport layer; a permission/login layer exists via `settings/permission/*` |

All paths in this document are **suffixes appended to the base URL** (e.g. `settings/controlMode`
→ `http://<ip>:22000/settings/controlMode`).

## Conventions used below

- **Returns** is the declared `Call<T>` body type. `opaque JSON (ResponseBody)` means the
  response body is untyped in the client (the GUI parses it ad-hoc) — its shape is *not*
  recoverable from the interface signature.
- **Request input** names the `@Body` model type (see the [Models](#request--response-models)
  appendix for field shapes), or `raw JSON/string body` for `@Body RequestBody`, or lists
  `@Query`/`@Path` parameters.
- The **path string is authoritative**. Where jadx emitted obfuscated duplicate methods for
  one `(method, path)`, the rows were collapsed; genuine overloads (same path, different body
  type) are kept and noted.
- Many resources expose **both `GET` (read) and `POST` (write/command)** on the same path.

## Connecting

`ConnectionAPI` models the controller session via a single `connection/state` resource:
`GET connection/state` reads connection status; `POST connection/state` (raw JSON body)
opens/closes the session. Controller identity is read via `properties/controllerType` and
`properties/cabinetType`.

## Robot mode & motion — quick map (and the 29999 ASCII equivalent)

The most useful part for driver work: how the GUI changes robot mode over HTTP, and the
equivalent command on the public ASCII protocol that this Viam module speaks.

| Goal | HTTP API (this document) | Body | 29999 ASCII equivalent |
|---|---|---|---|
| Enable servos | `POST settings/controlMode` | `{"controlMode":"enable"}` | `EnableRobot()` |
| Disable servos | `POST settings/controlMode` | `{"controlMode":"disable"}` | `DisableRobot()` |
| Enter drag / freedrive | `POST settings/controlMode` | `{"controlMode":"drag"}` | `StartDrag()` |
| Exit drag | `POST settings/controlMode` | `{"controlMode":"enable"}` | `StopDrag()` |
| Drag-teach style | `POST settings/setDragTeachMode` | `{"setDragTeachMode":"alldrag" \| "gesture" \| "translation" \| "disable"}` | no 1:1 — `StartDrag()` + `DragSensivity(...)` tuning |
| Drag sensitivity | `settings/function/dragSensivity` | — | `DragSensivity(index,value)` |
| Clear alarms | `POST interface/clearAlarms` | — | `ClearError()` |
| Jog | `POST interface/jogMode` | raw JSON | `MoveJog(...)` |
| Joint-space move | `POST interface/jointMovJ` | `JointJPoint` | `MovJ(joint={…})` |
| Cartesian move | `POST interface/movJ` | `MovePoint` | `MovJ(pose={…})` |
| Emergency stop | `Panel`/`interface` (see `EmergencyStopValue`) | `EmergencyStopValue` | `EmergencyStop(mode)` |
| Read mode / status | not exposed here (PropertiesApi is config + alarm lists only) | — | `RobotMode()` + feedback port `30004` |

> Note: `settings/controlMode` `"drag"`/`"enable"`/`"disable"` is the single GUI call that
> enables, disables, and enters/exits drag mode. On the ASCII side those are four separate
> commands (`EnableRobot`/`DisableRobot`/`StartDrag`/`StopDrag`). There is **no live
> robot-mode read** in this HTTP surface — the GUI reads mode/joints/pose from the realtime
> feedback stream, mirroring this module's `feedback.go` (port 30004).

## Endpoint reference

The following sections enumerate every endpoint of each Retrofit interface, in source order.


## InterfaceAPI — `interface/*`

Runtime robot interface commands: jogging, joint/Cartesian moves, homing, alarm/collision recovery, joint brakes, I/O (DI/DO/extend), grippers, conveyor encoders, force-sensor calibration, kinematics (forward/inverse), path playback/track, coordinate-system selection, e-skin, and device/network config (WiFi/AP, ethernet, bus mode).

| Method | Path | Request input | Returns | Java method |
|--------|------|---------------|---------|-------------|
| POST | `interface/apiUpdate` | ApiUpdateValue | opaque JSON (ResponseBody) | apiUpdate |
| POST | `interface/language` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | changeLanguage |
| POST | `interface/toolCoordinate` | CoordinateIndex | opaque JSON (ResponseBody) | changeToolCoordinate |
| POST | `interface/userCoordinate` | CoordinateIndex | opaque JSON (ResponseBody) | changeUserCoordinate |
| POST | `interface/setAP` | APData | opaque JSON (ResponseBody) | changeWifi |
| POST | `interface/clearAlarms` | — | opaque JSON (ResponseBody) | clearAlarms |
| GET | `interface/ClearElecSkinPopup` | — | opaque JSON (ResponseBody) | clearElecSkinPopup |
| POST | `process/elecSkin/reset` | — | opaque JSON (ResponseBody) | elecSkinReset |
| POST | `interface/coordinate` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | exchangeCoordinateType |
| POST | `interface/jogMode` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | exchangeJogMode |
| POST | `interface/logExportUSB` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | exportLogToUsb |
| POST | `interface/forwardCal` | ForwardCalData | opaque JSON (ResponseBody) | forwardCal |
| GET | `interface/setAPStatus` | — | opaque JSON (ResponseBody) | getAPStatus |
| GET | `interface/bus` | — | opaque JSON (ResponseBody) | getBusMode |
| POST | `interface/readCurrentEncoder` | ConveyorIndex | typed: CurrentEncoderValue | getCurrentEncoder |
| POST | `interface/readSensorEncoder` | ConveyorIndex | typed: CurrentSensorValue | getCurrentSensor |
| POST | `interface/getDIMode` | DIIndex | typed: DIModeValue | getDIMode |
| GET | `interface/dynamicData` | — | opaque JSON (ResponseBody) | getDynamicData |
| GET | `interface/ethernet` | — | opaque JSON (ResponseBody) | getEthernet |
| GET | `interface/logExportUSB` | — | opaque JSON (ResponseBody) | getExportLogToUsbState |
| GET | `interface/6axisJointBrake` | — | opaque JSON (ResponseBody) | getJointBrakeInfo |
| GET | `interface/jointMovJ` | — | opaque JSON (ResponseBody) | getJointMovJ |
| GET | `interface/movJ` | — | opaque JSON (ResponseBody) | getMovJ |
| GET | `interface/movL` | — | opaque JSON (ResponseBody) | getMovL |
| GET | `interface/debugReTrace` | — | opaque JSON (ResponseBody) | getPathPlaybackStatus |
| GET | `interface/resetElecSkin` | — | opaque JSON (ResponseBody) | getResetElecSkin |
| GET | `interface/getRobotError` | — | opaque JSON (ResponseBody) | getRobotError |
| GET | `interface/handleSafeCheckSumError` | — | opaque JSON (ResponseBody) | getSafeCheckSumError |
| GET | `interface/sixForceOffset` | — | opaque JSON (ResponseBody) | getSixForceOffset |
| GET | `interface/sixForceOrg` | — | opaque JSON (ResponseBody) | getSixForceOrg |
| POST | `interface/toolAlign` | — | opaque JSON (ResponseBody) | getToolAlign |
| GET | `interface/recurrentTrack` | — | opaque JSON (ResponseBody) | getTrackStatus |
| GET | `interface/setAP` | — | opaque JSON (ResponseBody) | getWifi |
| POST | `interface/handleSafeCheckSumError` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | handleSafeCheckSumError |
| POST | `interface/hardresetElecSkin` | — | opaque JSON (ResponseBody) | hardResetElecSkin |
| GET | `interface/hasElecSkin` | — | opaque JSON (ResponseBody) | hasElecSkin |
| POST | `interface/inverseCal` | InverseCalData | opaque JSON (ResponseBody) | inverseCal |
| POST | `interface/jointMovJ` | JointJPoint | opaque JSON (ResponseBody) | jointMovJ |
| POST | `interface/movJ` | MovePoint | opaque JSON (ResponseBody) | movJ |
| POST | `interface/movL` | MovePoint | opaque JSON (ResponseBody) | movL |
| POST | `interface/resetCollision` | — | opaque JSON (ResponseBody) | resetCollision |
| POST | `interface/resetElecSkin` | — | opaque JSON (ResponseBody) | resetElecSkin |
| POST | `interface/restartUpgradeService` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | restartUpgradeService |
| POST | `interface/auxGo` | RunPointForAux | opaque JSON (ResponseBody) | runToPointForAux |
| POST | `interface/setAPStatus` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | setAPStatus |
| POST | `interface/bus` | raw JSON/string body (RequestBody) | opaque JSON (ResponseBody) | setBusMode |
| POST | `interface/setDIMode` | DIMode | opaque JSON (ResponseBody) | setDIMode |
| POST | `interface/setDIValue` | DIValue | opaque JSON (ResponseBody) | setDIValue |
| POST | `interface/outputs` | ExchangeOutputs | opaque JSON (ResponseBody) | setDO |
| POST | `interface/dynamicData` | DynamicData | opaque JSON (ResponseBody) | setDynamicData |
| POST | `interface/ethernet` | EthernetData | opaque JSON (ResponseBody) | setEthernet |
| POST | `interface/extendIO` | ExtendOutputs | opaque JSON (ResponseBody) | setExtendIO |
| POST | `interface/hitbotGripper` | GripperValue | opaque JSON (ResponseBody) | setHitbotGripper |
| POST | `interface/goHome` | HomeValue | opaque JSON (ResponseBody) | setHome |
| POST | `interface/6axisJointBrake` | JointBrake _(overload: also accepts raw RequestBody)_ | opaque JSON (ResponseBody) | setJointBrakeInfo |
| POST | `interface/listenSensor` | ListenSensorValue | opaque JSON (ResponseBody) | setListenSensor |
| POST | `interface/debugReTrace` | InterfaceCMD | opaque JSON (ResponseBody) | setPathPlayback |
| POST | `interface/powerControl` | PowerControlValue | opaque JSON (ResponseBody) | setPowerControl |
| POST | `interface/robotiqGripper` | GripperValue | opaque JSON (ResponseBody) | setRobotIQGripper |
| POST | `interface/sixForceCaliX` | SixForceCalibrateValue | opaque JSON (ResponseBody) | sixForceCaliX |
| POST | `interface/sixForceCaliY` | SixForceCalibrateValue | opaque JSON (ResponseBody) | sixForceCaliY |
| POST | `interface/sixForceCaliZ` | SixForceCalibrateValue | opaque JSON (ResponseBody) | sixForceCaliZ |
| POST | `interface/recurrentTrack` | TrackPathParams | opaque JSON (ResponseBody) | trackPath |
| POST | `interface/updateFile` | — | opaque JSON (ResponseBody) | updateFile |

_64 rows / 65 method declarations (the two `setJointBrakeInfo` overloads on `POST interface/6axisJointBrake` are collapsed into one row; their `@Body` types genuinely differ — `JointBrake` vs raw `RequestBody`)._

### Notable for robot control

- **Jogging** — `POST interface/jogMode` (raw body) starts/stops jog motion; `POST interface/coordinate` (raw body, `exchangeCoordinateType`) selects the jog frame.
- **Joint move** — `POST interface/jointMovJ` (`JointJPoint`) commands a joint-space move; `GET interface/jointMovJ` reads back its current/last state.
- **Cartesian moves** — `POST interface/movJ` (`MovePoint`, joint-interpolated to a Cartesian target) and `POST interface/movL` (`MovePoint`, linear); each has a `GET` counterpart for state readback.
- **Homing** — `POST interface/goHome` (`HomeValue`, `setHome`).
- **Alarm / fault recovery** — `POST interface/clearAlarms` (no body), `GET interface/getRobotError`, `POST interface/resetCollision`, and the safety-checksum pair `GET`/`POST interface/handleSafeCheckSumError`.
- **Joint brakes** — `GET interface/6axisJointBrake` (read brake info) and `POST interface/6axisJointBrake` (`JointBrake` or raw body) to engage/release brakes; safety-critical when manually moving joints with power off.
- **Power** — `POST interface/powerControl` (`PowerControlValue`) for arm power on/off.
- **Auxiliary motion** — `POST interface/auxGo` (`RunPointForAux`, `runToPointForAux`).
- **Path playback / track replay** — `POST`/`GET interface/debugReTrace` (`InterfaceCMD`) and `POST`/`GET interface/recurrentTrack` (`TrackPathParams`).
- **Kinematics** — `POST interface/forwardCal` (`ForwardCalData`) and `POST interface/inverseCal` (`InverseCalData`).

> Note: this interface exposes no dedicated emergency-stop or enable/disable endpoint. The closest safety/power controls are `interface/powerControl`, `interface/clearAlarms`, `interface/resetCollision`, the joint-brake endpoints, and the safe-checksum handlers.

## SettingAPI — `settings/*`

Controller settings and function configuration: control mode (enable/disable/drag), drag-teach, coordinate systems (tool/user/teach), payload/load identification, IO function modes (DI/DO/AI/AO/tool/global), safety and collision detection, gripper/six-force end effectors, permissions/login, playback parameters, and product/system info.

> Returns are opaque JSON (`Call<ResponseBody>`) unless a concrete model/`Boolean` type is shown.
> Rows are listed in source-file order. The PATH string is authoritative.

| Method | Path | Request input | Returns | Java method |
|--------|------|---------------|---------|-------------|
| POST | `settings/function/normalVectorCal` | raw JSON/string body (`RequestBody`) | opaque JSON | calWall |
| POST | `settings/function/workZoneCal` | raw JSON/string body (`RequestBody`) | opaque JSON | calWorkZone |
| POST | `settings/calcInstall` | `CalibrationData` | opaque JSON | calcInstall |
| POST | `settings/permission/changeDefault` | raw JSON/string body (`RequestBody`) | opaque JSON | changeDefaultUser |
| POST | `settings/permission/changeName` | raw JSON/string body (`RequestBody`) | opaque JSON | changeName |
| POST | `settings/permission/changePassword` | raw JSON/string body (`RequestBody`) | opaque JSON | changePassword |
| POST | `settings/permission/changePassword` | raw JSON/string body (`RequestBody`) | opaque JSON | changePermissionName |
| POST | `settings/permission/changePassword` | raw JSON/string body (`RequestBody`) | opaque JSON | changePermissionPassword |
| POST | `project/teachFileDelete` | raw string body (`String`) | `Boolean` | deleteTeachFile |
| POST | `settings/permission/enablePermission` | raw JSON/string body (`RequestBody`) | opaque JSON | enablePermission |
| POST | `settings/permission/enablePassword` | raw JSON/string body (`RequestBody`) | opaque JSON | enableUserPassword |
| POST | `settings/params/exportPack` | `ExportImportInfo` | opaque JSON | exportPack |
| GET | `settings/function/adminPassword` | (none) | opaque JSON | getAdminPassword |
| GET | `settings/autoIdentifyResult` | (none) | opaque JSON | getAutoIdentifyRes |
| GET | `settings/autoIdentifyState` | (none) | opaque JSON | getAutoIdentifyState |
| GET | `settings/function/autoManual` | (none) | opaque JSON | getAutoManual |
| GET | `settings/function/autoManualPassword` | (none) | opaque JSON | getAutoManualPassword |
| GET | `settings/function/autoManualSwitch` | (none) | opaque JSON | getAutoManualSwitch |
| GET | `settings/function/ccboxVoltage` | (none) | opaque JSON | getCCBoxVoltage |
| GET | `settings/productInfo/controllerLocale` | (none) | opaque JSON | getControllerLocale |
| GET | `settings/version` | (none) | opaque JSON | getControllerVersion |
| GET | `settings/coordinate/tool` | (none) | `List<ToolCoordinate>` | getCoordinateTools |
| GET | `settings/coordinate/user` | (none) | `List<UserCoordinate>` | getCoordinateUsers |
| GET | `settings/function/customPoint` | (none) | opaque JSON | getCustomPoint |
| GET | `settings/function/workModeDI` | (none) | opaque JSON | getDIWorkMode |
| GET | `settings/function/workModeDO` | (none) | opaque JSON | getDOWorkMode |
| GET | `settings/function/dragSensivity` | (none) | opaque JSON | getDragParams |
| GET | `settings/function/endAI` | (none) | opaque JSON | getEndAI |
| GET | `settings/productInfo/expireInfo` | (none) | opaque JSON | getExpireInfo |
| GET | `settings/params/getExportParams` | (none) | opaque JSON | getExportParams |
| GET | `settings/function/gpioAI` | (none) | opaque JSON | getGlobalAI |
| GET | `settings/function/gpioAO` | (none) | opaque JSON | getGlobalAO |
| GET | `settings/productInfo/hardwareInfo` | (none) | opaque JSON | getHardwareInfo |
| GET | `settings/function/collisionDect` | (none) | opaque JSON | getHitSafeSwitch |
| GET | `settings/teach/inch` | (none) | opaque JSON | getInchParams |
| GET | `settings/function/install` | (none) | opaque JSON | getInstallParams |
| GET | `settings/function/ioCtrl` | (none) | opaque JSON | getIoCtrl |
| GET | `properties/jointLimits` | (none) | opaque JSON | getJointLimits |
| GET | `settings/function/loadConfig` | (none) | opaque JSON | getLoadConfig |
| GET | `settings/function/loadParams` | (none) | opaque JSON | getLoadParamsNew |
| GET | `settings/function/maxCollisionStrength` | (none) | opaque JSON | getMaxCollisionStrength |
| GET | `settings/function/modbusCtrl` | (none) | opaque JSON | getModbusCtrl |
| POST | `settings/params/getPackInfo` | raw JSON/string body (`RequestBody`) | opaque JSON | getPackInfo |
| GET | `settings/function/packPoint` | (none) | opaque JSON | getPackPoint |
| GET | `settings/function/pathDeviation` | (none) | opaque JSON | getPathDeviation |
| GET | `settings/permission/config` | (none) | opaque JSON | getPermissionConfig |
| POST | `settings/playback/arch` | (none) | `List<ArchSetting>` | getPlaybackArches |
| GET | `settings/playback/coordinate` | (none) | `PlaybackCoordinate` | getPlaybackCoordinate |
| GET | `settings/playback/joint` | (none) | `PlaybackJoint` | getPlaybackJoint |
| GET | `settings/function/reTraceParams` | (none) | opaque JSON | getReTraceParamNew |
| GET | `settings/function/remoteControl` | (none) | opaque JSON | getRemoteControl |
| GET | `settings/function/remoteSwitch` | (none) | opaque JSON | getRemoteSwitch |
| GET | `settings/function/runButtonModeE6` | (none) | opaque JSON | getRunButtonModeE6 |
| GET | `settings/function/safeControllerParams` | (none) | opaque JSON | getSafeControllerParams |
| GET | `settings/function/safeLimit` | (none) | opaque JSON | getSafeLimit |
| GET | `settings/function/safeSignal` | (none) | opaque JSON | getSafeSignal |
| GET | `settings/function/securityConstraint` | (none) | `SecurityConstraint` | getSecurityConstraint |
| GET | `settings/simulatedAxies` | (none) | opaque JSON | getSimulatedAxiesState |
| GET | `settings/function/ccboxVoltage` | (none) | opaque JSON | getSupplyVoltage |
| GET | `settings/function/switchAutoAction` | (none) | opaque JSON | getSwitchAutoAction |
| GET | `settings/function/scriptRunAction` | (none) | opaque JSON | getSwitchRunAction |
| GET | `settings/permission/switch` | (none) | opaque JSON | getSwitchUserPermissions |
| GET | `settings/systemTime` | (none) | opaque JSON | getSystemTime |
| GET | `settings/teach/coordinate` | (none) | `TeachCoordinate` | getTeachCoordinate |
| GET | `settings/teach/joint` | (none) | `TeachJoint` | getTeachJoint |
| GET | `settings/function/threeSwitchMode` | (none) | opaque JSON | getThreeSwitchMode |
| GET | `settings/function/threeSwitchStatus` | (none) | opaque JSON | getThreeSwitchStatus |
| GET | `settings/function/toolModeDO` | (none) | opaque JSON | getToolDOMode |
| GET | `settings/function/toolMode` | (none) | opaque JSON | getToolWorkMode |
| POST | `settings/function/torqueDifference` | `TorqueDifferenceValue` | opaque JSON | getTorqueDifference |
| GET | `settings/permission/userList` | (none) | opaque JSON | getUserPermissionsList |
| GET | `settings/function/virtualWall` | (none) | opaque JSON | getVirtualWall |
| GET | `settings/workTimeRec` | (none) | opaque JSON | getWorkTime |
| GET | `settings/function/workZone` | (none) | opaque JSON | getWorkZone |
| POST | `settings/params/importPack` | `ExportImportInfo` | opaque JSON | importPack |
| POST | `settings/function/loadAutoIdentify` | `List<double[]>` | opaque JSON | loadAutoIdentify |
| POST | `settings/function/loadCheck` | (none) | opaque JSON | loadCheck |
| POST | `settings/function/loadIdentifyMovJ` | `LoadIdentifyMovJPoint` | opaque JSON | loadIdentifyMovJ |
| POST | `settings/function/recordLoad` | raw JSON/string body (`RequestBody`) | opaque JSON | recordLoad |
| POST | `settings/workTimeRec` | (none) | opaque JSON | resetWorkTime |
| POST | `settings/function/adminPassword` | raw JSON/string body (`RequestBody`) | opaque JSON | setAdminPassword |
| POST | `settings/autoIdentify` | `AutoIdentifyValue` | opaque JSON | setAutoIdentify |
| POST | `settings/autoIdentifyStart` | (none) | opaque JSON | setAutoIdentifyStart |
| POST | `settings/autoIdentifyStop` | (none) | opaque JSON | setAutoIdentifyStop |
| POST | `settings/function/autoManual` | raw JSON/string body (`RequestBody`) | opaque JSON | setAutoManual |
| POST | `settings/function/autoManualPassword` | raw JSON/string body (`RequestBody`) | opaque JSON | setAutoManualPassword |
| POST | `settings/function/autoManualSwitch` | raw JSON/string body (`RequestBody`) | opaque JSON | setAutoManualSwitch |
| POST | `settings/function/ccboxVoltage` | raw JSON/string body (`RequestBody`) | opaque JSON | setCCBoxVoltage |
| POST | `settings/controlMode` | `ControlMode` | opaque JSON | setControlMode |
| POST | `settings/coordinate/tool` | `List<ToolCoordinate>` | opaque JSON | setCoordinateTools |
| POST | `settings/coordinate/user` | `List<UserCoordinate>` | opaque JSON | setCoordinateUsers |
| POST | `settings/function/customPoint` | `PointValue` | opaque JSON | setCustomPoint |
| POST | `settings/function/workModeDI` | raw JSON/string body (`RequestBody`) | opaque JSON | setDIWorkMode |
| POST | `settings/function/workModeDO` | raw JSON/string body (`RequestBody`) | opaque JSON | setDOWorkMode |
| POST | `settings/function/dragSensivity` | `DragData` | opaque JSON | setDragParams |
| POST | `settings/setDragTeachMode` | `DragTeachMode` | opaque JSON | setDragTeachMode |
| POST | `settings/function/elecSkin` | `ElectronicSkinValue` | opaque JSON | setElectronicSkin |
| POST | `settings/function/elecSkinParams` | `ElectronicSkinParams` | opaque JSON | setElectronicSkinParams |
| POST | `settings/function/endAI` | raw JSON/string body (`RequestBody`) | opaque JSON | setEndAI |
| POST | `settings/productInfo/expireInfo` | raw JSON/string body (`RequestBody`) | opaque JSON | setExpireInfo |
| POST | `settings/function/generalSafeSetting` | (none) | opaque JSON | setGeneralSafeSetting |
| POST | `settings/function/gpioAI` | raw JSON/string body (`RequestBody`) | opaque JSON | setGlobalAI |
| POST | `settings/function/gpioAO` | raw JSON/string body (`RequestBody`) | opaque JSON | setGlobalAO |
| POST | `settings/function/gpioAOValue` | raw JSON/string body (`RequestBody`) | opaque JSON | setGlobalAOValue |
| POST | `settings/productInfo/hardwareInfo` | raw JSON/string body (`RequestBody`) | opaque JSON | setHardwareInfo |
| POST | `settings/function/collisionDect` | `CollisionDect` | opaque JSON | setHitSafeSwitch |
| POST | `settings/function/hitbotGripperEnable` | `GripperState` | opaque JSON | setHitbotGripperEnable |
| POST | `settings/teach/inch` | `InchValue` | opaque JSON | setInchParams |
| POST | `settings/function/install` | `InstallValue` | opaque JSON | setInstallParams |
| POST | `settings/function/ioCtrl` | `IoCtrlData` | opaque JSON | setIoCtrl |
| POST | `properties/jointLimits` | raw JSON/string body (`RequestBody`) | opaque JSON | setJointLimits |
| POST | `settings/function/loadConfig` | `List<LoadConfig>` | opaque JSON | setLoadConfig |
| POST | `settings/function/setLoad` | `LoadValue` | opaque JSON | setLoadParams |
| POST | `settings/function/setLoad` | `LoadValueFourAxis` | opaque JSON | setLoadParamsFourAxis |
| POST | `settings/function/loadParams` | raw JSON/string body (`RequestBody`) | opaque JSON | setLoadParamsNew |
| POST | `settings/function/loadParams` | `LoadValueFourAxis` | opaque JSON | setLoadParamsNewFourAxis |
| POST | `settings/function/modbusCtrl` | `ModbusCtrlData` | opaque JSON | setModbusCtrl |
| POST | `settings/permission/config` | raw JSON/string body (`RequestBody`) | opaque JSON | setPermissionConfig |
| POST | `settings/playback/arch` | `List<ArchSetting>` | opaque JSON | setPlaybackArches |
| POST | `settings/playback/coordinate` | `PlaybackCoordinate` | `PlaybackCoordinate` | setPlaybackCoordinate |
| POST | `settings/playback/joint` | `PlaybackJoint` | `PlaybackJoint` | setPlaybackJoint |
| POST | `settings/common` | `Ratio` | opaque JSON | setRatio |
| POST | `settings/function/setReTraceParam` | raw JSON/string body (`RequestBody`) | opaque JSON | setReTraceParam |
| POST | `settings/function/reTraceParams` | `ReTraceParam` | opaque JSON | setReTraceParamNew |
| POST | `settings/function/remoteControl` | `RemoteControl` | opaque JSON | setRemoteControl |
| POST | `settings/function/remoteSwitch` | raw JSON/string body (`RequestBody`) | opaque JSON | setRemoteSwitch |
| POST | `settings/function/robotiqGripperEnable` | `GripperState` | opaque JSON | setRobotiqGripperEnable |
| POST | `settings/function/robotiqSixForce` | `RobotiqSixForce` | opaque JSON | setRobotiqSixForce |
| POST | `settings/function/runButtonModeE6` | raw JSON/string body (`RequestBody`) | opaque JSON | setRunButtonModeE6 |
| POST | `settings/function/safeControllerParams` | raw JSON/string body (`RequestBody`) | opaque JSON | setSafeControllerParams |
| POST | `settings/function/safeLimit` | raw JSON/string body (`RequestBody`) | opaque JSON | setSafeLimit |
| POST | `settings/function/safeSignal` | `SafeSignalData` | opaque JSON | setSafeSignal |
| POST | `settings/function/safeSignal` | raw JSON/string body (`RequestBody`) | opaque JSON | setSafeSignal (overload) |
| POST | `settings/function/securityConstraint` | `SecurityConstraint` | `SecurityConstraint` | setSecurityConstraint |
| POST | `settings/simulatedAxies` | `JointSimulated` | opaque JSON | setSimulatedAxiesState |
| POST | `settings/function/sixForceHome` | `SixForceHomeValue` | opaque JSON | setSixForceHome |
| POST | `settings/function/sixForceParams` | (none) | opaque JSON | setSixForceParams |
| POST | `settings/function/ccboxVoltage` | raw JSON/string body (`RequestBody`) | opaque JSON | setSupplyVoltage |
| POST | `settings/function/switchAutoAction` | raw JSON/string body (`RequestBody`) | opaque JSON | setSwitchAutoAction |
| POST | `settings/function/scriptRunAction` | raw JSON/string body (`RequestBody`) | opaque JSON | setSwitchRunAction |
| POST | `settings/permission/switch` | raw JSON/string body (`RequestBody`) | opaque JSON | setSwitchUserPermissions |
| POST | `settings/systemTime` | `SystemTime` | opaque JSON | setSystemTime |
| POST | `settings/systemTime` | raw JSON/string body (`RequestBody`) | opaque JSON | setSystemTime (overload) |
| POST | `settings/teach/coordinate` | `TeachCoordinate` | `TeachCoordinate` | setTeachCoordinate |
| POST | `settings/teach/joint` | `TeachJoint` | `TeachJoint` | setTeachJoint |
| POST | `settings/function/threeSwitchMode` | raw JSON/string body (`RequestBody`) | opaque JSON | setThreeSwitchMode |
| POST | `settings/function/threeSwitchStatus` | raw JSON/string body (`RequestBody`) | opaque JSON | setThreeSwitchStatus |
| POST | `settings/function/toolModeDO` | raw JSON/string body (`RequestBody`) | opaque JSON | setToolDOMode |
| POST | `settings/function/toolMode` | raw JSON/string body (`RequestBody`) | opaque JSON | setToolMode |
| POST | `settings/permission/userList` | raw JSON/string body (`RequestBody`) | opaque JSON | setUserPermissionsList |
| POST | `project/teachFileUpdate` | raw string body (`String`) | `Boolean` | updateTeachFile |
| POST | `settings/permission/login` | raw JSON/string body (`RequestBody`) | opaque JSON | userLogin |
| POST | `settings/permission/logout` | (none) | opaque JSON | userLogout |

**Row count: 153 endpoints** (154 method declarations in the source; the obfuscated near-duplicate `setDragTeacMode` was collapsed into `setDragTeachMode` — see notes).

### Notable for robot mode / motion control

- **`POST settings/controlMode` (`ControlMode`)** — the primary mode switch: enable / disable / drag-teach state of the robot. This is the closest HTTP analog to the dashboard `EnableRobot`/`DisableRobot` plus drag toggle.
- **`POST settings/setDragTeachMode` (`DragTeachMode`)** — explicitly toggles drag-teaching (hand-guiding) mode. Method `setDragTeachMode` (the obfuscated near-duplicate `setDragTeacMode`, same path+body, was collapsed).
- **`GET`/`POST settings/function/dragSensivity` (`DragData`)** — drag-teach sensitivity / responsiveness tuning (`getDragParams` / `setDragParams`).
- **Payload / load (mass & CoM identification):**
  - `POST settings/function/setLoad` — `setLoadParams` (`LoadValue`) and overload `setLoadParamsFourAxis` (`LoadValueFourAxis`).
  - `GET`/`POST settings/function/loadParams` — `getLoadParamsNew` / `setLoadParamsNew` (raw body) and overload `setLoadParamsNewFourAxis` (`LoadValueFourAxis`).
  - `GET`/`POST settings/function/loadConfig` (`List<LoadConfig>`) — `getLoadConfig` / `setLoadConfig`.
  - `POST settings/function/recordLoad` (raw body), `POST settings/function/loadCheck` (no body).
  - Auto load-identification routine: `POST settings/function/loadAutoIdentify` (`List<double[]>`), `POST settings/function/loadIdentifyMovJ` (`LoadIdentifyMovJPoint` — moves the arm during identification), plus `settings/autoIdentify`, `settings/autoIdentifyStart`, `settings/autoIdentifyStop`, `GET settings/autoIdentifyState`, `GET settings/autoIdentifyResult`.
- **Coordinate / tool / user frames:**
  - `GET`/`POST settings/coordinate/tool` (`List<ToolCoordinate>`) and `settings/coordinate/user` (`List<UserCoordinate>`) — tool and user coordinate system tables.
  - `settings/teach/coordinate` (`TeachCoordinate`), `settings/teach/joint` (`TeachJoint`), `settings/teach/inch` (`InchValue`) — teach-pendant jog frame / step (inch) settings.
  - `settings/playback/coordinate` (`PlaybackCoordinate`), `settings/playback/joint` (`PlaybackJoint`), `settings/playback/arch` (`List<ArchSetting>`) — playback motion frame / joint / arch (gantry-style) parameters.
  - `settings/calcInstall` (`CalibrationData`) and `GET`/`POST settings/function/install` (`InstallValue`) — mounting/installation orientation (affects gravity compensation and motion).
- **Collision / safety (motion-relevant guarding):**
  - `GET`/`POST settings/function/collisionDect` (`CollisionDect`) — collision-detection switch (`getHitSafeSwitch` / `setHitSafeSwitch`).
  - `GET settings/function/maxCollisionStrength` — collision sensitivity ceiling.
  - `GET`/`POST settings/function/securityConstraint` (`SecurityConstraint`), `settings/function/safeLimit`, `settings/function/safeControllerParams`, `settings/function/safeSignal` (`SafeSignalData` + raw overload), `POST settings/function/generalSafeSetting` (no body).
  - Virtual wall / work zone: `GET settings/function/virtualWall`, `GET`/`POST settings/function/workZone`, `POST settings/function/workZoneCal`, `POST settings/function/normalVectorCal` (`calWall`).
  - `GET settings/function/pathDeviation` — path-deviation monitoring.
  - `POST settings/function/torqueDifference` (`TorqueDifferenceValue`) — joint torque-difference threshold.
- **Joint limits:** `GET`/`POST properties/jointLimits` (raw body) — software joint-angle limits.
- **Gripper / end effector:** `settings/function/hitbotGripperEnable` and `settings/function/robotiqGripperEnable` (`GripperState`); `settings/function/robotiqSixForce` (`RobotiqSixForce`), `settings/function/sixForceHome` (`SixForceHomeValue`), `settings/function/sixForceParams`; electronic skin `settings/function/elecSkin` (`ElectronicSkinValue`) / `settings/function/elecSkinParams` (`ElectronicSkinParams`).
- **Re-trace / path replay:** `GET`/`POST settings/function/reTraceParams` (`ReTraceParam`) and legacy `POST settings/function/setReTraceParam` (raw body).
- **Remote / auto-manual mode switching (run-state relevant):** `settings/function/remoteControl` (`RemoteControl`), `settings/function/remoteSwitch`, `settings/function/autoManual`, `settings/function/autoManualSwitch`, `settings/function/switchAutoAction`, `settings/function/scriptRunAction`, `settings/function/runButtonModeE6`, `settings/function/threeSwitchMode` / `threeSwitchStatus`.
- **Speed ratio:** `POST settings/common` (`Ratio`) — global speed/ratio factor (`setRatio`).
- **Simulated axes:** `GET`/`POST settings/simulatedAxies` (`JointSimulated`) — run joints in simulation (no physical motion).

### Notes on duplicates / overloads

- **Collapsed (jadx obfuscation artifact):** `setDragTeacMode` and `setDragTeachMode` are identical (`POST settings/setDragTeachMode`, body `DragTeachMode`); only the human-readable `setDragTeachMode` is kept above. Net: 154 source methods → 153 rows.
- **Genuine overloads (same method+path, differing `@Body` type) — both kept:**
  - `settings/function/setLoad`: `setLoadParams` (`LoadValue`) vs `setLoadParamsFourAxis` (`LoadValueFourAxis`).
  - `settings/function/loadParams`: `setLoadParamsNew` (raw `RequestBody`) vs `setLoadParamsNewFourAxis` (`LoadValueFourAxis`).
  - `settings/function/safeSignal`: `setSafeSignal(SafeSignalData)` vs `setSafeSignal(RequestBody)` (same Java name, true overload).
  - `settings/systemTime`: `setSystemTime(SystemTime)` vs `setSystemTime(RequestBody)` (same Java name, true overload).
- **Distinct human-readable methods sharing one path/body (alias methods, all kept — not obfuscation):**
  - `settings/permission/changePassword` is the target of three methods: `changePassword`, `changePermissionName`, `changePermissionPassword` (all `RequestBody`).
  - `settings/function/ccboxVoltage` GET is hit by both `getCCBoxVoltage` and `getSupplyVoltage`; POST by both `setCCBoxVoltage` and `setSupplyVoltage` (all `RequestBody`).
- **Non-`settings` paths present in this interface:** `project/teachFileDelete` / `project/teachFileUpdate` (raw `String` body, return `Boolean`) and `properties/jointLimits` (GET/POST).

## DebuggerAPI

Purpose: drive the on-controller script debugger — start/stop/step/suspend a debug session, manage breakpoints, read debugger state and global variables, and configure axis simulation.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| POST | `debugger/breakPoint` | none | `DebuggerStatus` | `deleteAllBreakpoint` |
| POST | `debugger/delb` | `@Body BreakPoint` | `DebuggerStatus` | `deleteBreakpoint` |
| GET | `debugger/state` | none | `DebuggerToolState` | `getDebuggerState` |
| GET | `debugger/state` | none | opaque JSON (ResponseBody) | `getDebuggerState2` |
| GET | `debugger/globalVar` | none | opaque JSON (ResponseBody) | `getGlobalVar` |
| GET | `debugger/breakPoint` | none | opaque JSON (ResponseBody) | `getRemoteBreakPoint` |
| GET | `debugger/runb` | none | opaque JSON (ResponseBody) | `getRunBStatus` |
| GET | `debugger/simulate/axes` | none | opaque JSON (ResponseBody) | `getSimulated` |
| GET | `debugger/step` | none | opaque JSON (ResponseBody) | `getStepStatus` |
| POST | `debugger/runb` | none | `DebuggerStatus` | `runB` |
| POST | `debugger/run` | none | `DebuggerStatus` | `runDebugger` |
| POST | `debugger/setb` | `@Body BreakPoint` | `DebuggerStatus` | `setBreakpoint` |
| POST | `debugger/simulate/axes` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setSimulated` |
| POST | `debugger/start` | `@Body DebuggerFile` | `DebuggerStatus` | `startDebugger` |
| POST | `debugger/stepIn` | none | opaque JSON (ResponseBody) | `stepIn` |
| POST | `debugger/stepOver` | none | `DebuggerStatus` | `stepOver` |
| POST | `debugger/stop` | none | `DebuggerStatus` | `stopDebugger` |
| POST | `debugger/suspend` | none | `DebuggerStatus` | `suspendDebugger` |
| POST | `debugger/unzipProject` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `unzipProject` |

Notes:
- `GET debugger/state` is declared twice with the same path but distinct method names and return types (`DebuggerToolState` vs `ResponseBody`) — both kept above; they are typed variants, not pure obfuscated duplicates.
- `debugger/breakPoint` is overloaded across verbs: `POST` deletes all breakpoints (`deleteAllBreakpoint`) while `GET` reads remote breakpoints (`getRemoteBreakPoint`).
- `debugger/runb` and `debugger/simulate/axes` each pair a `GET` status read with a `POST` action.

## PropertiesApi

Purpose: read robot/controller property documents (config defaults, controller/cabinet type, advanced-function and suppression config) and read alarm/warning lists; write advanced-function and suppression config.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| GET | `properties/advancedFunction` | none | opaque JSON (ResponseBody) | `getAdvancedFunctionData` |
| GET | `properties/alarmController` | none | `List<AlertJsonData>` | `getAlarmController` |
| GET | `properties/alarmServo` | none | `List<AlertJsonData>` | `getAlarmServo` |
| GET | `properties/cabinetType` | none | opaque JSON (ResponseBody) | `getCabinetType` |
| GET | `properties/controllerType` | none | opaque JSON (ResponseBody) | `getControllerType` |
| GET | `properties/default` | none | opaque JSON (ResponseBody) | `getDefaultJson` |
| GET | `properties/suppressionConfig` | none | opaque JSON (ResponseBody) | `getSuppressionConfig` |
| GET | `properties/warning` | none | `List<AlertJsonData>` | `getWarningJson` |
| POST | `properties/advancedFunction` | `@Body AdvancedFunctionData` | opaque JSON (ResponseBody) | `setAdvancedFunctionData` |
| POST | `properties/advancedFunction` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setAdvancedFunctionData` |
| POST | `properties/suppressionConfig` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setSuppressionConfig` |

Endpoints relevant to a driver (robot status / alarm / mode reads):
- `GET properties/alarmController` (`getAlarmController`) — controller-side alarm list, typed as `List<AlertJsonData>`.
- `GET properties/alarmServo` (`getAlarmServo`) — servo-side alarm list, typed as `List<AlertJsonData>`.
- `GET properties/warning` (`getWarningJson`) — active warning list, typed as `List<AlertJsonData>`.
- `GET properties/controllerType` (`getControllerType`) — identifies the controller (model/mode identification).
- `GET properties/cabinetType` (`getCabinetType`) — identifies the control cabinet.

Note: this interface is mostly property/config documents plus alarm/warning lists. There is no explicit live pose / joint-angle / robot-mode endpoint here — alarms and warnings are the only directly status-relevant reads. (`setAdvancedFunctionData` is overloaded with a typed-body and a raw-body variant on the same `POST properties/advancedFunction` path.)

## ProtocolAPI

Purpose: low-level protocol pass-through — exchange a protocol command/response with the controller (typed, alternate-typed, and raw variants on one endpoint) and read current alarm/warning state.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| POST | `protocol/exchange` | `@Body ExchangeSendData` | `ExchangeAssistanceReceiveData` | `exchangeAssistanceData` |
| POST | `protocol/exchange` | `@Body ExchangeSendData` | `ExchangeReceiveData` | `exchangeData` |
| POST | `protocol/exchange` | `@Body ExchangeSendData` | opaque JSON (ResponseBody) | `exchangeDataRaw` |
| GET | `protocol/getAlarm` | none | opaque JSON (ResponseBody) | `getAlarm` |
| GET | `protocol/getWarning` | none | opaque JSON (ResponseBody) | `getWarning` |

Notes:
- `POST protocol/exchange` has three declarations with identical method and request input (`@Body ExchangeSendData`) but different return types (`ExchangeAssistanceReceiveData`, `ExchangeReceiveData`, `ResponseBody`) — all kept as distinct typed result variants.
- `GET protocol/getAlarm` / `GET protocol/getWarning` are status-relevant reads (current alarm / warning state) for a driver.

## ConnectionAPI

Connect / disconnect the studio to a controller and read connection state.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| GET | `connection/state` | none | opaque JSON (ResponseBody) | `getConnectionState` |
| POST | `connection/state` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setConnectionState` |

- **Connect/state flow:** A single `connection/state` resource models the session.
  `GET connection/state` reads the current connection status (connected /
  disconnected to the controller). `POST connection/state` mutates it — i.e. the
  client establishes or tears down the controller session by writing a state
  payload (the connect/disconnect request) to the same path. The body shape is an
  opaque raw JSON body, so the exact connect parameters (e.g. address/port/flag)
  are not visible from the interface signature alone.

## CalibrateAPI

Calibrate tool/user coordinate frames, home position, and left/right-hand config.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| POST | `calibrate/coordinate/tool/pose` | raw string body (`@Body String`) | `CalibrateCoordinatePose` | `setCalibrateCoordinatePose` |
| POST | `calibrate/coordinate/tool/position` | raw string body (`@Body String`) | `CalibrateCoordinatePosition` | `setCalibrateCoordinatePosition` |
| POST | `calibrate/coordinate/tool` | raw string body (`@Body String`) | `CalibrateCoordinateTool` | `setCalibrateCoordinateTool` |
| POST | `calibrate/coordinate/user` | raw JSON/string body (`@Body RequestBody`) | `CalibrateCoordinateUser` | `setCalibrateCoordinateUser` |
| POST | `calibrate/coordinate/userLine` | raw string body (`@Body String`) | `CalibrateCoordinateUser` | `setCalibrateCoordinateUserLine` |
| POST | `calibrate/coordinate/userDot` | raw string body (`@Body String`) | `CalibrateCoordinateUser` | `setCalibrateCoordinateUserPoint` |
| POST | `calibrate/home` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setCalibrateHome` |
| POST | `calibrate/home` | none (empty body) | opaque JSON (ResponseBody) | `setCalibrateHomeNullBody` |
| POST | `calibrate/homeSingleAxis` | `SingleAxis` model | opaque JSON (ResponseBody) | `setCalibrateHomeSingle` |
| POST | `calibrate/leftRightHand` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setCalibrateLeftRightHand` |

> Note: `calibrate/home` is exposed by two distinct methods — one with a request
> body (`setCalibrateHome`) and one with no body (`setCalibrateHomeNullBody`). Both
> are retained because they are genuinely different call shapes, not obfuscated
> duplicates.

## DobotPlusAPI

Manage "DobotPlus" plugins/extensions: install/uninstall, hotkeys, ports, IO control.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| POST | `dobotPlus/Call` | `DobotPlusCall` model | opaque JSON (ResponseBody) | `dobotPlusCall` |
| GET | `dobotPlus/hotKey` | none | `DobotPlusHotKey` | `getDobotPlusHotKey` |
| GET | `dobotPlus/getKeys` | none | opaque JSON (ResponseBody) | `getDobotPlusHotKeyList` |
| GET | `dobotPlus/list` | none | opaque JSON (ResponseBody) | `getDobotPlusList` |
| GET | `dobotPlus/getPorts` | none | opaque JSON (ResponseBody) | `getDobotPlusPorts` |
| GET | `dobotPlus/setIOCtrl` | none | opaque JSON (ResponseBody) | `getIOCtrl` |
| POST | `dobotPlus/install` | `DobotPlusItem` model | opaque JSON (ResponseBody) | `installDobotPlus` |
| POST | `dobotPlus/hotKey` | `DobotPlusHotKey` model | opaque JSON (ResponseBody) | `setDobotPlusHotKey` |
| POST | `dobotPlus/setIOCtrl` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `setIOCtrl` |
| POST | `dobotPlus/uninstall` | `DobotPlusItem` model | opaque JSON (ResponseBody) | `uninstallDobotPlus` |

> Note: `dobotPlus/setIOCtrl` and `dobotPlus/hotKey` each appear under both GET and
> POST verbs (read vs. write); they are distinct endpoints, not duplicates.

## PanelAPI

Teach-pendant / control-panel actions: emergency stop, jog, auto/manual, mode switch.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| POST | `panel/emergencyStop` | `EmergencyStopValue` model | `PanelStatus` | `emergencyStop` |
| GET | `panel/emergencyStop` | none | opaque JSON (ResponseBody) | `getEmergencyStop` |
| POST | `panel/jog` | `JOGButtonValue` model | `PanelStatus` | `setJog` |
| POST | `panel/autoManual` | `AutoManualValue` model | `PanelStatus` | `setPanelAutoManual` |
| POST | `panel/threeSwitch` | `ThreeSwitchValue` model | opaque JSON (ResponseBody) | `setThreeSwitch` |

## ProcessAPI

Read process-feature toggles (auxiliary joint, electronic skin).

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| GET | `process/auxJoint/switch` | none | opaque JSON (ResponseBody) | `getAuxJointSwitch` |
| GET | `process/elecSkin/switch` | none | opaque JSON (ResponseBody) | `getSkinSwitch` |

## ProjectAPI

Push teach-file updates to the current project.

| Method | Path | Request input | Returns | Java method |
| --- | --- | --- | --- | --- |
| POST | `project/teachFileUpdate` | raw JSON/string body (`@Body RequestBody`) | opaque JSON (ResponseBody) | `teachFileUpdate` |

## Request / Response Models

Models are Gson-serialized POJOs from `cc.dobot.dobotstudio.http.model`. Fields below are JSON keys (the Java field name; Gson uses the field name verbatim unless an `@SerializedName` is noted); constants are the `public static final` allowed string/boolean values. All 91 model classes are documented below.

### Models most relevant to robot mode/motion

- [`ControlMode`](#controlmode) — enable / disable / drag mode switch.
- [`DragTeachMode`](#dragteachmode) — drag-teach mode (alldrag / disable / gesture / translation).
- [`DragTeachValue`](#dragteachvalue) — drag-teach enable + per-axis direction + mode.
- [`JointValue`](#jointvalue) — J1..J6 joint angles (single point).
- [`JointJPoint`](#jointjpoint) — joint target array for a joint move.
- [`MovePoint`](#movepoint) — generic move target (joint + pose + tool/user frame).
- [`RunPointForAux`](#runpointforaux) — full Cartesian/aux move target (x,y,z,a,b,c,...).
- [`LoadIdentifyMovJPoint`](#loadidentifymovjpoint) — joint move used during load identification.
- [`LoadConfig`](#loadconfig) / [`LoadValue`](#loadvalue) / [`LoadValueFourAxis`](#loadvaluefouraxis) — payload mass/center-of-mass.
- [`EmergencyStopValue`](#emergencystopvalue) — e-stop boolean.
- [`CoordinateTool`](#coordinatetool) / [`CoordinateUser`](#coordinateuser) — tool/user coordinate frame definitions.
- [`CoordinateIndex`](#coordinateindex) / [`ConveyorIndex`](#conveyorindex) — frame/conveyor selection index.
- [`JointBrake`](#jointbrake) — joint brake control (empty marker class).
- [`JointSimulated`](#jointsimulated) — per-joint simulated flags.
- [`HomeValue`](#homevalue) / [`PowerControlValue`](#powercontrolvalue) — homing / power enable.
- [`PlaybackJoint`](#playbackjoint) / [`PlaybackCoordinate`](#playbackcoordinate) / [`TeachJoint`](#teachjoint) / [`TeachCoordinate`](#teachcoordinate) — velocity/accel/jerk motion profiles.
- [`ExchangeSendData`](#exchangesenddata) / [`ExchangeReceiveData`](#exchangereceivedata) / [`ExchangeAssistanceReceiveData`](#exchangeassistancereceivedata) — the main control/feedback exchange payloads (carry control mode, coordinate frame, joint/cartesian coords, alarms, e-stop, etc.).

---

### `APData`
- `busy`: boolean
- `enable`: boolean
- `passWd`: String
- `ssid`: String

### `ApiUpdateValue`
- `path`: String

### `Arch`
- `enable`: boolean (default `true`)
- Nested inner class `params` (not a top-level model): `startHeight`: int (default `0`), `endHeight`: int (default `0`), `zLimit`: int (default `0`)

### `AutoIdentifyValue`
- `name`: String
- `value`: boolean

### `AutoManualPassword`
- `code`: String
- `mSwitch`: boolean — `@SerializedName("switch")` (resolves from `SVGConstants.SVG_SWITCH_TAG = "switch"`)

### `AutoManualValue`
- `value`: String (default `"auto"`)
- Constants: `"auto"`, `"manual"`

### `BreakPoint`
- `path`: String (default `""`)
- `line`: int (default `0`)

### `CalibrateCoordinatePose`
- `result`: boolean (default `false`)
- `coordinate`: double[] (default `{0.0, 0.0, 0.0}`)

### `CalibrateCoordinatePosition`
- `result`: boolean (default `false`)
- `coordinate`: double[] (default `{0.0, 0.0, 0.0}`)
- `deviation`: double (default `0.0`)

### `CalibrateCoordinateTool`
- `result`: boolean (default `false`)
- `coordinate`: double[] (default `{0.0, 0.0, 0.0, 0.0}`)

### `CalibrateCoordinateUser`
- `result`: boolean (default `false`)
- `coordinate`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)

### `ControlMode`
- `controlMode`: String (default `"disable"`)
- Constants: `"disable"`, `"drag"`, `"enable"`

### `ConveyorIndex`
- `conveyor_index`: int

### `CoordinateIndex`
- `index`: int

### `CoordinateTool`
- `enable`: boolean (default `false`)
- `params`: int[] (default `{0, 0, 0, 0, 0, 0}`)

### `CoordinateUser`
- `enable`: boolean (default `false`)
- `params`: int[] (default `{0, 0, 0, 0, 0, 0}`)

### `CurrentEncoderValue`
- `value`: double

### `CurrentSensorValue`
- `value`: double

### `DebuggerFile`
- `project`: String (default `"DefaultPro"`)

### `DebuggerStatus`
- `status`: boolean (default `true`)

### `DebuggerToolState`
- `value`: String (default `"stopped"`)
- `prjname`: String (default `""`)
- `type`: String (default `"Lua"`)
- Constants: `RUNNING = "running"`, `STOP = "stopped"`

### `DIIndex`
- `number`: int
- `start`: int
- `type`: int

### `DIMode`
- `index`: int
- `mode`: int
- `type`: int

### `DIModeValue`
- `status`: boolean
- `val`: int[]

### `DIValue`
- `index`: int
- `type`: int
- `value`: int

### `DobotPlusCall`
- `args`: int[]
- `function`: String
- `path`: String
- `timeout`: int

### `DobotPlusHotKey`
- `longPress`: String
- `name`: String
- `press1`: String
- `press2`: String

### `DobotPlusItem`
- `name`: String

### `DragTeachMode`
- `setDragTeachMode`: String (default `"disable"`)
- Constants: `"alldrag"`, `"disable"`, `"gesture"`, `"translation"`

### `DragTeachValue`
- `direction`: boolean[]
- `mode`: int
- `status`: boolean

### `DynamicData`
- `value`: boolean (default `false`)

### `ElectronicSkinParams`
- `avoidAcc`: int
- `avoidDistance`: int
- `avoidVel`: int
- `mode`: int
- `resumeAcc`: int
- `resumeVel`: int
- `toolSkinCH`: int

### `ElectronicSkinValue`
- `value`: boolean

### `EmergencyStopValue`
- `value`: boolean (default `false`)

### `EthernetData`
- `dhcp`: boolean
- `gateway`: String
- `ip`: String
- `netmask`: String

### `ExchangeAssistanceReceiveData`
- `alarmId`: int
- `alarms`: String[][]
- `armOrientation`: int
- `autoManual`: String
- `auxJoint`: double[]
- `cartesianCoordinate`: double[]
- `controlMode`: String
- `controlParams`: float[]
- `coordinate`: String
- `dragMode`: boolean
- `dragPlayback`: Boolean (boxed)
- `dragTeach`: DragTeachValue
- `dragTrack`: boolean
- `emergencyStop`: Boolean (boxed)
- `endAI`: double[]
- `endDI`: int[]
- `endDO`: int[]
- `extendDI`: int[][]
- `extendDO`: int[][]
- `forceSensorData`: double[][]
- `forceSensorStatus`: int
- `forceSensorSwitch`: boolean
- `gpioAI`: double[]
- `gpioAO`: double[]
- `inputs`: int[]
- `isAlarmUpdate`: boolean
- `isMotion`: boolean
- `isSafeRun`: int
- `isSafeSuspend`: int
- `isWarningUpdate`: boolean
- `jogMode`: String
- `jointBrake`: int[]
- `jointCoordinate`: double[]
- `jointCurrent`: float[]
- `jointTemp`: float[]
- `jointVoltage`: float[]
- `ledStatus`: int
- `outputs`: int[]
- `pointSignal`: int
- `prjState`: String
- `protectiveStop`: boolean
- `rdnCoordinate`: int[]
- `recoveryMode`: boolean
- `reducedMode`: boolean
- `remoteControl`: String
- `remoteRun`: boolean
- `safeCheck`: SafeCheck
- `skinProximity`: boolean
- `skinValue`: float[]
- `speedRatio`: int
- `toolCoordinate`: int
- `toolMode`: ToolModeData
- `userCoordinate`: int
- `warning`: int
- `warningList`: int[]
- `powerState`: String (default `"on"` — `DebugKt.DEBUG_PROPERTY_VALUE_ON`)
- `isCollision`: int (default `0`)
- `skinCollison`: int (default `0`)
- `pastSkinCollison`: int (default `0`)
- `safeDO`: int[] (default `{2, 3}`)
- `safeDI`: int[] (default `{6, 2}`)

### `ExchangeOutputs`
- `type`: int
- `value`: int[]

### `ExchangeReceiveData`
- `alarms`: String[][]
- `armOrientation`: int
- `autoManual`: String
- `cartesianCoordinate`: float[]
- `controlMode`: String
- `coordinate`: String
- `inputs`: float[]
- `jogMode`: String
- `jointCoordinate`: float[]
- `outputs`: float[]
- `toolCoordinate`: int
- `userCoordinate`: int

### `ExchangeSendData`
- `controlMode`: String (default `"disable"`)
- `coordinate`: String (default `"joint"` — `COORDINATE_JOINT`)
- `dragMode`: String (default `"disable"`)
- `jogMode`: String (default `"jog"` — `JOG_MODE_JOG`)
- `toolCoordinate`: int (default `0`)
- `userCoordinate`: int (default `0`)
- `alarms`: boolean (default `true`)
- `outputs`: ExchangeOutputs (default `new ExchangeOutputs()`)
- `extendDO`: ExtendOutputs (default `new ExtendOutputs()`)
- `hardware`: boolean (default `true`)
- `modbusIO`: boolean (default `false`)
- Constants: control mode `"disable"` / `"drag"` / `"enable"`; coordinate `"cartesian"` / `"joint"` / `"tool"`; drag mode `"disable"` / `"enable"`; jog mode `"jog"` / `"step"`

### `ExportImportInfo`
- `packTime`: String
- `packageName`: String
- `project`: List<String>
- `systemParams`: List<String>

### `ExtendOutputs`
- `enable`: boolean
- `value`: int[][]

### `ForwardCalData`
- `joint`: double[]
- `tool`: int
- `user`: int

### `GripperState`
- `enable`: boolean

### `GripperValue`
- `value`: boolean

### `HitSafeValue`
- `value`: boolean

### `HomeValue`
- `value`: boolean

### `InchValue`
- `angle`: float
- `distance`: float

### `InstallValue`
- `rotationAngle`: float
- `slopeAngle`: float

### `InterfaceCMD`
- `addr`: String
- `cmd`: String
- Constants: `CMD_START = "start"`, `CMD_STOP = "stop"`, `MOVE_J = "moveJ"`, `MOVE_L = "moveL"`

### `InverseCalData`
- `coordinate`: double[]
- `jointNear`: double[]
- `tool`: int
- `useJointNear`: boolean
- `user`: int

### `JOGButtonValue`
- `posBtns`: boolean[] (default `{false, false, false, false, false, false}`)
- `negBtns`: boolean[] (default `{false, false, false, false, false, false}`)

### `JointBrake`
- (no fields — empty marker/placeholder class)

### `JointJPoint`
- `joint`: double[]
- `value`: boolean

### `JointSimulated`
- `joint1Simulated`: boolean
- `joint2Simulated`: boolean
- `joint3Simulated`: boolean
- `joint4Simulated`: boolean
- `joint5Simulated`: boolean
- `joint6Simulated`: boolean

### `JointValue`
- `J1`: double
- `J2`: double
- `J3`: double
- `J4`: double
- `J5`: double
- `J6`: double

### `ListenSensorValue`
- `cmd`: String
- `conveyor_index`: int

### `LoadConfig`
- `centerX`: double
- `centerY`: double
- `centerZ`: double
- `loadValue`: double
- `name`: String (default `""`)
- `points`: List<JointValue>

### `LoadIdentifyMovJPoint`
- `globalSpeed`: int
- `joint`: double[]
- `speedRatio`: double
- `teachSpeed`: TeachJoint
- `value`: boolean

### `LoadValue`
- `centerX`: double
- `centerY`: double
- `centerZ`: double
- `isCheck`: boolean
- `loadValue`: double
- `name`: String (default `""`)

### `LoadValueFourAxis`
- `inertiaX`: double
- `inertiaY`: double
- `inertiaZ`: double
- `loadValue`: double

### `MovePoint`
- `joint`: double[]
- `pose`: double[]
- `tool`: int
- `user`: int
- `value`: boolean

### `PanelStatus`
- `status`: boolean

### `PathDeviationData`
- `deviation`: int
- `pausedPoint`: double[]

### `PermissionConfig`
- `config`: PermissionConfigParams
- `level`: int

### `PermissionConfigParams`
- `IO`: int
- `Modbus`: int
- `advancedSettings`: int
- `autoManualSettings`: int
- `baseFunc`: int
- `buttonSettings`: int
- `communication`: int
- `coordinate`: int
- `drag`: int
- `globalVariable`: int
- `homeCalibration`: int
- `installation`: int
- `jog`: int
- `loadParameters`: int
- `log`: int
- `motionParameters`: int
- `pluginOperations`: int
- `postureSettings`: int
- `powerSettings`: int
- `projectFileOperations`: int
- `projectStateOpertions`: int
- `remoteMode`: int
- `security`: int
- `systemTime`: int
- `teachPointOperations`: int
- `trajectoryPlayback`: int

### `PlaybackCoordinate`
- `velocity`: double[] (default `{0.0, 0.0}`)
- `acceleration`: double[] (default `{0.0, 0.0}`)
- `jerk`: double[] (default `{0.0, 0.0}`)

### `PlaybackJoint`
- `velocity`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)
- `acceleration`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)
- `jerk`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)

### `PowerControlValue`
- `value`: boolean

### `ProjectData`
- `isLocalProject`: boolean
- `projectName`: String
- `time`: long

### `Ratio`
- `ratio`: int (default `0`)

### `RemoteControl`
- `mode`: String
- `name`: String
- Constants: `REMOTE_CONTROL_IO = "io"`, `REMOTE_CONTROL_MODBUS = "modbus"`, `REMOTE_CONTROL_TCP_IP = "tcp"`, `REMOTE_CONTROL_TP = "tp"`

### `RobotiqSixForce`
- `enableSixForce`: boolean

### `RunPointForAux`
- `a`: double
- `armOrientation`: String
- `aux1`: double
- `aux2`: double
- `aux3`: double
- `b`: double
- `c`: double
- `cfg`: int
- `d`: int
- `mode`: String
- `n`: int
- `r`: int
- `tool`: int
- `user`: int
- `value`: boolean
- `x`: double
- `y`: double
- `z`: double

### `SafeCheck`
- `isError`: boolean
- `safeCheckSum`: String

### `SafeConfig`
- `forceConstr`: String
- `monentumConstr`: String
- `powerConstr`: String
- `velConstr`: String

### `SafeParams`
- `safeconfig`: SafeConfig

### `SecurityConstraint`
- `reducedVelocity`: double[] (default `{0.0, 0.0}`)

### `SixForceCalibrateValue`
- `cartesianCoord`: float[]
- `orgValue`: float[]

### `SixForceHomeValue`
- `cartesianCoord`: float[] (default `{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}`)
- `orgValue`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)

### `SystemTime`
- `date`: String
- `time`: String
- `timeZone`: String

### `TeachCoordinate`
- `velocity`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)
- `acceleration`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)
- `jerk`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)

### `TeachJoint`
- `velocity`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)
- `acceleration`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)
- `jerk`: double[] (default `{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}`)

### `ThreeSwitchValue`
- `value`: boolean (default `false`)
- Constants: `PRESSED = true`, `RELEASED = false`

### `ToolModeData`
- `mode1`: int
- `mode2`: int

### `TorqueDifferenceValue`
- `index`: int
- `joint`: double[]
- `type`: int

### `TrackPathParams`
- `getPos`: boolean (default `true`)

### `UserLevelForND`
- `jobNumber`: String
- `level`: int
- `password`: String
- `username`: String

### `UserPermissionValue`
- `enablePassword`: boolean
- `level`: int
- `name`: String
- `password`: String

### `WallData`
- `DI_Index`: int
- `DO_Index`: int
- `alias`: String
- `enable`: boolean
- `isElbowLimited`: boolean
- `point1`: float[]
- `point2`: float[]
- `point3`: float[]
- `radius`: int
- `type`: int

### `WorkTime`
- `powerOnTime`: WorkTimeDetail
- `servoEnableTime`: WorkTimeDetail

### `WorkTimeDetail`
- `day`: long
- `hour`: long
- `min`: long
- `sec`: long
