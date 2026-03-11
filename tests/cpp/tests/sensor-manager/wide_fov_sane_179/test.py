import time
from pylmcp import Object
from pylmcp.server import Server
from pylmcp.uxas import SensorManager, UxASConfig

bridge_cfg = UxASConfig()
bridge_cfg += SensorManager()

with Server(bridge_cfg=bridge_cfg) as server:
    try:
        gimbal = Object(
            class_name='GimbalConfiguration',
            PayloadID=10,
            MinElevation=-150.0,
            MaxElevation=-10.0,
            IsElevationClamped=True,
            ContainedPayloadList=[20],
            randomize=True
        )
        camera = Object(
            class_name='CameraConfiguration',
            PayloadID=20,
            VideoStreamHorizontalResolution=1920,
            VideoStreamVerticalResolution=1080,
            SupportedWavelengthBand=1,
            FieldOfViewMode=1,
            DiscreteHorizontalFieldOfViewList=[179.0],
            randomize=True
        )
        vehicle_config = Object(
            class_name='AirVehicleConfiguration',
            ID=400,
            NominalAltitude=1000.0,
            PayloadConfigurationList=[gimbal, camera],
            randomize=True
        )
        server.send_msg(vehicle_config)
        time.sleep(0.2)
        req = Object(
            class_name='task.FootprintRequest',
            FootprintRequestID=1, VehicleID=400,
            EligibleWavelengths=[1],
            GroundSampleDistances=[5.0],
            AglAltitudes=[1000.0],
            ElevationAngles=[-45.0],
            randomize=True
        )
        server.send_msg(Object(
            class_name='task.SensorFootprintRequests',
            RequestID=100, Footprints=[req], randomize=True))
        msg = server.wait_for_msg(
            descriptor='uxas.messages.task.SensorFootprintResponse', timeout=5.0)
        fp = msg.obj['Footprints'][0]
        assert fp['AchievedGSD'] > 0
        assert fp['WidthCenter'] > 0, f"WidthCenter={fp['WidthCenter']} should be positive for HFOV=179"
        print(f"HFOV=179: elev={fp['GimbalElevation']}  GSD={fp['AchievedGSD']:.4f}  WidthCenter={fp['WidthCenter']:.2f}")
        print("OK")
    finally:
        pass
