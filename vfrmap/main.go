package main

//go:generate go-bindata -pkg main -o bindata.go -modtime 1 -prefix html html

// build: GOOS=windows GOARCH=amd64 go build -o vfrmap.exe github.com/MarphXL/msfs2020-go/vfrmap

import (
	// https://github.com/lian/msfs2020-go
	"encoding/json"
	"flag"
	"fmt"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"
	"unsafe"

	// https://github.com/lian/msfs2020-go
	"github.com/MarphXL/msfs2020-go/simconnect"
	"github.com/MarphXL/msfs2020-go/vfrmap/html/leafletjs"
	"github.com/MarphXL/msfs2020-go/vfrmap/websockets"

	// https://github.com/tarm/serial
	//"github.com/MarphXL/msfs2020-go/vfrmap/serial"
)
// Marph was here
type Report struct {
	simconnect.RecvSimobjectDataByType
		AirspeedIndicated                       float64    `name:"AIRSPEED INDICATED" unit:"knots"` // Flight Instrumentation:Indicated airspeed
		AirspeedTrue                            float64    `name:"AIRSPEED TRUE" unit:"knots"` // Flight Instrumentation:True airspeed
		AiTrafficAssignedRunway                 [256]byte  `name:"AI TRAFFIC ASSIGNED RUNWAY" unit:"string"` // AI Controlled Aircraft:Assigned runway name (for example: "32R"). See 
		AiTrafficState                          [256]byte  `name:"AI TRAFFIC STATE" unit:"string"` // AI Controlled Aircraft:English string describing an AI object's state.
		AiTrafficToairport                      [256]byte  `name:"AI TRAFFIC TOAIRPORT" unit:"string"` // AI Controlled Aircraft:ICAO code of the destination airport in the cur
		AmbientTemperature                      float64    `name:"AMBIENT TEMPERATURE" unit:"celsius"` // Environment:Ambient temperature
		AtcFlightNumber                         [256]byte  `name:"ATC FLIGHT NUMBER" unit:"string (6)"` // String:Flight Number used by ATC
		AtcId                                   [256]byte  `name:"ATC ID" unit:"string (10)"` // String:ID used by ATC
		AtcModel                                [256]byte  `name:"ATC MODEL" unit:"string (10)"` // String:Model used by ATC
		AtcType                                 [256]byte  `name:"ATC TYPE" unit:"string (30)"` // String:Type used by ATC
		AutopilotAvailable                      float64       `name:"AUTOPILOT AVAILABLE" unit:"bool"` // Autopilot:Available flag
		AutopilotFlightDirectorActive           float64       `name:"AUTOPILOT FLIGHT DIRECTOR ACTIVE" unit:"bool"` // Autopilot:Flight director active
		AutopilotMaster                         float64       `name:"AUTOPILOT MASTER" unit:"bool"` // Autopilot:On/off flag
		AvatarModeIsAttached                    float64       `name:"AVATAR MODE IS ATTACHED" unit:"bool"` // Avatar Mode:Sets/gets the attachment state of the user avatar.
		AvionicsMasterSwitch                    float64       `name:"AVIONICS MASTER SWITCH" unit:"bool"` // Avionics:Avionics switch state
		BarometerPressure                       float64    `name:"BAROMETER PRESSURE" unit:"millibars"` // Environment:Barometric pressure
		BrakeIndicator                          float64    `name:"BRAKE INDICATOR" unit:"position"` // Avionics:Brake on indication [0.0: Off, 1.0: Full Brakes]
		BrakeParkingIndicator                   float64       `name:"BRAKE PARKING INDICATOR" unit:"bool"` // Avionics:Parking brake indicator
		BrakeParkingPosition                    float64       `name:"BRAKE PARKING POSITION" unit:"bool"` // Avionics:Parking brake on [0: Off, 1.0: Full Parking Brake]
		ElectricalBatteryLoad                   float64    `name:"ELECTRICAL BATTERY LOAD" unit:"ampere"` // Miscellaneous Systems:Battery load
		ElectricalMainBusVoltage                float64    `name:"ELECTRICAL MAIN BUS VOLTAGE" unit:"volts"` // Miscellaneous Systems:Main bus voltage
		ElectricalMasterBattery                 float64       `name:"ELECTRICAL MASTER BATTERY" unit:"bool"` // Miscellaneous Systems:Battery switch position
		ElevatorDeflectionPct                   float64    `name:"ELEVATOR DEFLECTION PCT" unit:"percent over 100"` // Avionics:Percent deflection
		ElevatorTrimPct                         float64    `name:"ELEVATOR TRIM PCT" unit:"percent over 100"` // Avionics:Percent elevator trim
		EmptyWeight                             float64    `name:"EMPTY WEIGHT" unit:"pounds"` // Miscellaneous:Empty weight of the aircraft
		EngMaxRpm                               float64    `name:"ENG MAX RPM" unit:"rpm"` // Engine:Maximum rpm
		FlapsAvailable                          float64       `name:"FLAPS AVAILABLE" unit:"bool"` // Avionics:True if flaps available
		FlapsHandleIndex                        float64    `name:"FLAPS HANDLE INDEX" unit:"number"` // Avionics:Index of current flap position
		FlapsHandlePercent                      float64    `name:"FLAPS HANDLE PERCENT" unit:"percent over 100"` // Avionics:Percent flap handle extended
		FlapsNumHandlePositions                 float64    `name:"FLAPS NUM HANDLE POSITIONS" unit:"number"` // Avionics:Number of flap positions
		FuelTotalCapacity                       float64    `name:"FUEL TOTAL CAPACITY" unit:"gallons"` // Fuel:Total capacity of the aircraft
		FuelTotalQuantity                       float64    `name:"FUEL TOTAL QUANTITY" unit:"gallons"` // Fuel:Current quantity in volume
		FuelTotalQuantityWeight                 float64    `name:"FUEL TOTAL QUANTITY WEIGHT" unit:"pounds"` // Fuel:Current total fuel weight of the aircraft
		GearTotalPctExtended                    float64    `name:"GEAR TOTAL PCT EXTENDED" unit:"percent"` // Landing Gear:Percent total gear extended
		GeneralEngStarterActive                 float64       `name:"GENERAL ENG STARTER ACTIVE" unit:"bool"` // Engine:True if engine starter is active
		GpsApproachAirportId                    [256]byte  `name:"GPS APPROACH AIRPORT ID" unit:"string"` // String:ID of airport
		GpsGroundSpeed                          float64    `name:"GPS GROUND SPEED" unit:"meters per second"` // Avionics:Current ground speed
		HeadingIndicator                        float64    `name:"HEADING INDICATOR" unit:"radians"` // Flight Instrumentation:Heading indicator (directional gyro) indication
		IndicatedAltitude                       float64    `name:"INDICATED ALTITUDE" unit:"feet"` // Flight Instrumentation:Altimeter indication
		LightBeaconOn                           float64       `name:"LIGHT BEACON ON" unit:"bool"` // Lights:Return true if the light is on.
		LightBrakeOn                            float64       `name:"LIGHT BRAKE ON" unit:"bool"` // Lights:Return true if the light is on.
		LightCabinOn                            float64       `name:"LIGHT CABIN ON" unit:"bool"` // Lights:Return true if the light is on.
		LightHeadOn                             float64       `name:"LIGHT HEAD ON" unit:"bool"` // Lights:Return true if the light is on.
		LightLandingOn                          float64       `name:"LIGHT LANDING ON" unit:"bool"` // Lights:Return true if the light is on.
		LightLogoOn                             float64       `name:"LIGHT LOGO ON" unit:"bool"` // Lights:Return true if the light is on.
		LightNavOn                              float64       `name:"LIGHT NAV ON" unit:"bool"` // Lights:Return true if the light is on.
		LightPanelOn                            float64       `name:"LIGHT PANEL ON" unit:"bool"` // Lights:Return true if the light is on.
		LightRecognitionOn                      float64       `name:"LIGHT RECOGNITION ON" unit:"bool"` // Lights:Return true if the light is on.
		LightStrobeOn                           float64       `name:"LIGHT STROBE ON" unit:"bool"` // Lights:Return true if the light is on.
		LightTaxiOn                             float64       `name:"LIGHT TAXI ON" unit:"bool"` // Lights:Return true if the light is on.
		LightWingOn                             float64       `name:"LIGHT WING ON" unit:"bool"` // Lights:Return true if the light is on.
		MagneticCompass                         float64    `name:"MAGNETIC COMPASS" unit:"degrees"` // Miscellaneous:Compass reading
		MasterIgnitionSwitch                    float64       `name:"MASTER IGNITION SWITCH" unit:"bool"` // Engine:Aircraft master ignition switch (grounds all engines magnetos)
		MaxGrossWeight                          float64    `name:"MAX GROSS WEIGHT" unit:"pounds"` // Miscellaneous:Maximum gross weight of the aircraft
		NumberOfEngines                         float64    `name:"NUMBER OF ENGINES" unit:"number"` // Engine:Number of engines (minimum 0, maximum 4)
		OverspeedWarning                        float64       `name:"OVERSPEED WARNING" unit:"bool"` // Flight Instrumentation:Overspeed warning state
		PlaneAltAboveGround                     float64    `name:"PLANE ALT ABOVE GROUND" unit:"feet"` // Position and Speed:Altitude above the surface
		PlaneHeadingDegreesTrue                 float64    `name:"PLANE HEADING DEGREES TRUE" unit:"radians"` // Position and Speed:Heading relative to true north, although the name m
		PlaneLatitude                           float64    `name:"PLANE LATITUDE" unit:"radians"` // Position and Speed:Latitude of aircraft, North is positive, South nega
		PlaneLongitude                          float64    `name:"PLANE LONGITUDE" unit:"radians"` // Position and Speed:Longitude of aircraft, East is positive, West negat
		RudderPosition                          float64    `name:"RUDDER POSITION" unit:"position"` // Avionics:Rudder input deflection [-1.0: Full Left, 1.0: Full Right]
		RudderTrim                              float64    `name:"RUDDER TRIM" unit:"radians"` // Avionics:Angle deflection
		RudderTrimPct                           float64    `name:"RUDDER TRIM PCT" unit:"percent over 100"` // Avionics:The trim position of the rudder. Zero is no trim.
		SimOnGround                             float64       `name:"SIM ON GROUND" unit:"bool"` // Position and Speed:On ground flag (To set, passing a non-zero argument
		SpoilersHandlePosition                  float64    `name:"SPOILERS HANDLE POSITION" unit:"position"` // Avionics:Spoiler handle position [0: Retracted, 1.0: Fully Extended]
		StallWarning                            float64       `name:"STALL WARNING" unit:"bool"` // Flight Instrumentation:Stall warning state

		Title                                   [256]byte  `name:"TITLE" unit:"variable length string"` // String:title from aircraft.cfg
		TotalWeight                             float64    `name:"TOTAL WEIGHT" unit:"pounds"` // Miscellaneous:Total weight of the aircraft
		TrailingEdgeFlapsLeftAngle              float64    `name:"TRAILING EDGE FLAPS LEFT ANGLE" unit:"radians"` // Avionics:Angle left trailing edge flap extended. Use TRAILING EDGE FLA
		TrailingEdgeFlapsRightAngle             float64    `name:"TRAILING EDGE FLAPS RIGHT ANGLE" unit:"radians"` // Avionics:Angle right trailing edge flap extended. Use TRAILING EDGE FL
		TransponderCode_i                       float64    `name:"TRANSPONDER CODE:index" unit:"bco16"` // Avionics:4-digit code
		UnlimitedFuel                           float64       `name:"UNLIMITED FUEL" unit:"bool"` // Fuel:Unlimited fuel flag
		VariometerRate                          float64    `name:"VARIOMETER RATE" unit:"feet per second"` // Miscellaneous:Variometer rate
		VerticalSpeed                           float64    `name:"VERTICAL SPEED" unit:"feet per second"` // Flight Instrumentation:Vertical speed indication

}

func (r *Report) RequestData(s *simconnect.SimConnect) {
	defineID := s.GetDefineID(r)
	requestID := defineID
	s.RequestDataOnSimObjectType(requestID, defineID, 0, simconnect.SIMOBJECT_TYPE_USER)
}

type TrafficReport struct {
	simconnect.RecvSimobjectDataByType
	AtcID           [64]byte `name:"ATC ID"`
	AtcFlightNumber [8]byte  `name:"ATC FLIGHT NUMBER"`
	Altitude        float64  `name:"PLANE ALTITUDE" unit:"feet"`
	Latitude        float64  `name:"PLANE LATITUDE" unit:"degrees"`
	Longitude       float64  `name:"PLANE LONGITUDE" unit:"degrees"`
	Heading         float64  `name:"PLANE HEADING DEGREES TRUE" unit:"degrees"`
}

func (r *TrafficReport) RequestData(s *simconnect.SimConnect) {
	defineID := s.GetDefineID(r)
	requestID := defineID
	s.RequestDataOnSimObjectType(requestID, defineID, 0, simconnect.SIMOBJECT_TYPE_AIRCRAFT)
}

func (r *TrafficReport) Inspect() string {
	return fmt.Sprintf(
		"%s GPS %.6f %.6f @ %.0f feet %.0f°",
		r.AtcID,
		r.Latitude,
		r.Longitude,
		r.Altitude,
		r.Heading,
	)
}

type TeleportRequest struct {
	simconnect.RecvSimobjectDataByType
	Latitude  float64 `name:"PLANE LATITUDE" unit:"degrees"`
	Longitude float64 `name:"PLANE LONGITUDE" unit:"degrees"`
	Altitude  float64 `name:"PLANE ALTITUDE" unit:"feet"`
}

func (r *TeleportRequest) SetData(s *simconnect.SimConnect) {
	defineID := s.GetDefineID(r)

	buf := [3]float64{
		r.Latitude,
		r.Longitude,
		r.Altitude,
	}

	size := simconnect.DWORD(3 * 8) // 2 * 8 bytes
	s.SetDataOnSimObject(defineID, simconnect.OBJECT_ID_USER, 0, 0, size, unsafe.Pointer(&buf[0]))
}

var buildVersion string
var buildTime string
var disableTeleport bool

var verbose bool
var httpListen string
// Marph Variables for serialwrite
var n int

func main() {
	flag.BoolVar(&verbose, "verbose", false, "verbose output")
	flag.StringVar(&httpListen, "listen", "0.0.0.0:9000", "http listen")
	flag.BoolVar(&disableTeleport, "disable-teleport", false, "disable teleport")
	flag.Parse()

	fmt.Printf("\nmsfs2020-go/vfrmap\n  readme: https://github.com/MarphXL/msfs2020-go/blob/master/vfrmap/README.md\n  issues: https://github.com/MarphXL/msfs2020-go/issues\n  version: %s (%s)\n\n", buildVersion, buildTime)

	exitSignal := make(chan os.Signal, 1)
	signal.Notify(exitSignal, os.Interrupt, syscall.SIGTERM)
	exePath, _ := os.Executable()

	ws := websockets.New()

	s, err := simconnect.New("msfs2020-go/vfrmap")
	if err != nil {
		panic(err)
	}
	fmt.Println("connected to flight simulator!")

	report := &Report{}
	err = s.RegisterDataDefinition(report)
	if err != nil {
		panic(err)
	}

	trafficReport := &TrafficReport{}
	err = s.RegisterDataDefinition(trafficReport)
	if err != nil {
		panic(err)
	}

	teleportReport := &TeleportRequest{}
	err = s.RegisterDataDefinition(teleportReport)
	if err != nil {
		panic(err)
	}

	//open serial-port
	//c := &serial.Config{Name: "COM31", Baud: 9600}
        //s1, err := serial.OpenPort(c)
        //if err != nil {
      //          panic(err)
        //}
	//fmt.Println("serial connected to Arduino!")

	eventSimStartID := s.GetEventID()
	//s.SubscribeToSystemEvent(eventSimStartID, "SimStart")
	//s.SubscribeToFacilities(simconnect.FACILITY_LIST_TYPE_AIRPORT, s.GetDefineID(&simconnect.DataFacilityAirport{}))
	//s.SubscribeToFacilities(simconnect.FACILITY_LIST_TYPE_WAYPOINT, s.GetDefineID(&simconnect.DataFacilityWaypoint{}))

	startupTextEventID := s.GetEventID()
	s.ShowText(simconnect.TEXT_TYPE_PRINT_WHITE, 15, startupTextEventID, "msfs2020-go/vfrmap connected")

	go func() {
		app := func(w http.ResponseWriter, r *http.Request) {
			w.Header().Set("Access-Control-Allow-Origin", "*")
			w.Header().Set("Cache-Control", "no-cache, no-store, must-revalidate")
			w.Header().Set("Pragma", "no-cache")
			w.Header().Set("Expires", "0")
			w.Header().Set("Content-Type", "text/html")

			filePath := filepath.Join(filepath.Dir(exePath), "index.html")

			if _, err = os.Stat(filePath); os.IsNotExist(err) {
				w.Write(MustAsset(filepath.Base(filePath)))
			} else {
				fmt.Println("use local", filePath)
				http.ServeFile(w, r, filePath)
			}
		}

		http.HandleFunc("/ws", ws.Serve)
		http.Handle("/leafletjs/", http.StripPrefix("/leafletjs/", leafletjs.FS{}))
		http.HandleFunc("/", app)
		//http.Handle("/", http.FileServer(http.Dir(".")))

		err := http.ListenAndServe(httpListen, nil)
		if err != nil {
			panic(err)
		}
	}()

	simconnectTick := time.NewTicker(100 * time.Millisecond)
	planePositionTick := time.NewTicker(200 * time.Millisecond)
	trafficPositionTick := time.NewTicker(10000 * time.Millisecond)

	for {
		select {
		case <-planePositionTick.C:
			report.RequestData(s)

		case <-trafficPositionTick.C:
			//fmt.Println("--------------------------------- REQUEST TRAFFIC --------------")
			//trafficReport.RequestData(s)
			//s.RequestFacilitiesList(simconnect.FACILITY_LIST_TYPE_AIRPORT, airportRequestID)
			//s.RequestFacilitiesList(simconnect.FACILITY_LIST_TYPE_WAYPOINT, waypointRequestID)

		case <-simconnectTick.C:
			ppData, r1, err := s.GetNextDispatch()

			if r1 < 0 {
				if uint32(r1) == simconnect.E_FAIL {
					// skip error, means no new messages?
					continue
				} else {
					panic(fmt.Errorf("GetNextDispatch error: %d %s", r1, err))
				}
			}

			recvInfo := *(*simconnect.Recv)(ppData)

			switch recvInfo.ID {
			case simconnect.RECV_ID_EXCEPTION:
				recvErr := *(*simconnect.RecvException)(ppData)
				fmt.Printf("SIMCONNECT_RECV_ID_EXCEPTION %#v\n", recvErr)

			case simconnect.RECV_ID_OPEN:
				recvOpen := *(*simconnect.RecvOpen)(ppData)
				fmt.Printf(
					"\nflight simulator info:\n  codename: %s\n  version: %d.%d (%d.%d)\n  simconnect: %d.%d (%d.%d)\n\n",
					recvOpen.ApplicationName,
					recvOpen.ApplicationVersionMajor,
					recvOpen.ApplicationVersionMinor,
					recvOpen.ApplicationBuildMajor,
					recvOpen.ApplicationBuildMinor,
					recvOpen.SimConnectVersionMajor,
					recvOpen.SimConnectVersionMinor,
					recvOpen.SimConnectBuildMajor,
					recvOpen.SimConnectBuildMinor,
				)

			case simconnect.RECV_ID_EVENT:
				recvEvent := *(*simconnect.RecvEvent)(ppData)

				switch recvEvent.EventID {
				case eventSimStartID:
					fmt.Println("EVENT: SimStart")
				case startupTextEventID:
					// ignore
				default:
					fmt.Println("unknown SIMCONNECT_RECV_ID_EVENT", recvEvent.EventID)
				}
			case simconnect.RECV_ID_WAYPOINT_LIST:
				waypointList := *(*simconnect.RecvFacilityWaypointList)(ppData)
				fmt.Printf("SIMCONNECT_RECV_ID_WAYPOINT_LIST %#v\n", waypointList)

			case simconnect.RECV_ID_AIRPORT_LIST:
				airportList := *(*simconnect.RecvFacilityAirportList)(ppData)
				fmt.Printf("SIMCONNECT_RECV_ID_AIRPORT_LIST %#v\n", airportList)

			case simconnect.RECV_ID_SIMOBJECT_DATA_BYTYPE:
				recvData := *(*simconnect.RecvSimobjectDataByType)(ppData)

				switch recvData.RequestID {
				case s.DefineMap["Report"]:
					report = (*Report)(ppData)

					if verbose {
						fmt.Printf("REPORT: %#v\n", report)
					}

					ws.Broadcast(map[string]interface{}{
						//"type":           "plane",
						//"latitude":       report.Latitude,
						//"longitude":      report.Longitude,
						//"altitude":       fmt.Sprintf("%.0f", report.Altitude),
						//"heading":        int(report.Heading),
						"ai_traffic_assigned_runway":  fmt.Sprintf("%s", report.AiTrafficAssignedRunway                 ),
						"ai_traffic_state":  fmt.Sprintf("%s", report.AiTrafficState                          ),
						"ai_traffic_toairport":  fmt.Sprintf("%s", report.AiTrafficToairport                      ),
						"airspeed_indicated":  fmt.Sprintf("%.1f", report.AirspeedIndicated                       ),
						"airspeed_true":  fmt.Sprintf("%.1f", report.AirspeedTrue                            ),
						"ambient_temperature":  fmt.Sprintf("%.1f", report.AmbientTemperature                      ),
						"atc_flight_number":  fmt.Sprintf("%s", report.AtcFlightNumber                         ),
						"atc_id":  fmt.Sprintf("%s", report.AtcId                                   ),
						"atc_model":  fmt.Sprintf("%s", report.AtcModel                                ),
						"atc_type":  fmt.Sprintf("%s", report.AtcType                                 ),
						"autopilot_available":  fmt.Sprintf("%.1f", report.AutopilotAvailable                      ),
						"autopilot_flight_director_active":  fmt.Sprintf("%.1f", report.AutopilotFlightDirectorActive           ),
						"autopilot_master":  fmt.Sprintf("%.1f", report.AutopilotMaster                         ),
						"avatar_mode_is_attached":  fmt.Sprintf("%.1f", report.AvatarModeIsAttached                    ),
						"avionics_master_switch":  fmt.Sprintf("%.1f", report.AvionicsMasterSwitch                    ),
						"barometer_pressure":  fmt.Sprintf("%.1f", report.BarometerPressure                       ),
						"brake_indicator":  fmt.Sprintf("%.1f", report.BrakeIndicator                          ),
						"brake_parking_indicator":  fmt.Sprintf("%.1f", report.BrakeParkingIndicator                   ),
						"brake_parking_position":  fmt.Sprintf("%.1f", report.BrakeParkingPosition                    ),
						"electrical_battery_load":  fmt.Sprintf("%.1f", report.ElectricalBatteryLoad                   ),
						"electrical_main_bus_voltage":  fmt.Sprintf("%.1f", report.ElectricalMainBusVoltage                ),
						"electrical_master_battery":  fmt.Sprintf("%.1f", report.ElectricalMasterBattery                 ),
						"elevator_deflection_pct":  fmt.Sprintf("%.1f", report.ElevatorDeflectionPct                   ),
						"elevator_trim_pct":  fmt.Sprintf("%.1f", report.ElevatorTrimPct                         ),
						"empty_weight":  fmt.Sprintf("%.1f", report.EmptyWeight                             ),
						"eng_max_rpm":  fmt.Sprintf("%.1f", report.EngMaxRpm                               ),
						"flaps_available":  fmt.Sprintf("%.1f", report.FlapsAvailable                          ),
						"flaps_handle_index":  fmt.Sprintf("%.1f", report.FlapsHandleIndex                        ),
						"flaps_handle_percent":  fmt.Sprintf("%.1f", report.FlapsHandlePercent                      ),
						"flaps_num_handle_positions":  fmt.Sprintf("%.1f", report.FlapsNumHandlePositions                 ),
						"fuel_total_capacity":  fmt.Sprintf("%.1f", report.FuelTotalCapacity                       ),
						"fuel_total_quantity":  fmt.Sprintf("%.1f", report.FuelTotalQuantity                       ),
						"fuel_total_quantity_weight":  fmt.Sprintf("%.1f", report.FuelTotalQuantityWeight                 ),
						"gear_total_pct_extended":  fmt.Sprintf("%.1f", report.GearTotalPctExtended                    ),
						"general_eng_starter_active":  fmt.Sprintf("%.1f", report.GeneralEngStarterActive                 ),
						"gps_approach_airport_id":  fmt.Sprintf("%s", report.GpsApproachAirportId                    ),
						"gps_ground_speed":  fmt.Sprintf("%.1f", report.GpsGroundSpeed                          ),
						"heading_indicator":  fmt.Sprintf("%.1f", report.HeadingIndicator                        ),
						"indicated_altitude":  fmt.Sprintf("%.1f", report.IndicatedAltitude                       ),
						"light_beacon_on":  fmt.Sprintf("%.1f", report.LightBeaconOn                           ),
						"light_brake_on":  fmt.Sprintf("%.1f", report.LightBrakeOn                            ),
						"light_cabin_on":  fmt.Sprintf("%.1f", report.LightCabinOn                            ),
						"light_head_on":  fmt.Sprintf("%.1f", report.LightHeadOn                             ),
						"light_landing_on":  fmt.Sprintf("%.1f", report.LightLandingOn                          ),
						"light_logo_on":  fmt.Sprintf("%.1f", report.LightLogoOn                             ),
						"light_nav_on":  fmt.Sprintf("%.1f", report.LightNavOn                              ),
						"light_panel_on":  fmt.Sprintf("%.1f", report.LightPanelOn                            ),
						"light_recognition_on":  fmt.Sprintf("%.1f", report.LightRecognitionOn                      ),
						"light_strobe_on":  fmt.Sprintf("%.1f", report.LightStrobeOn                           ),
						"light_taxi_on":  fmt.Sprintf("%.1f", report.LightTaxiOn                             ),
						"light_wing_on":  fmt.Sprintf("%.1f", report.LightWingOn                             ),
						"magnetic_compass":  fmt.Sprintf("%.1f", report.MagneticCompass                         ),
						"master_ignition_switch":  fmt.Sprintf("%.1f", report.MasterIgnitionSwitch                    ),
						"max_gross_weight":  fmt.Sprintf("%.1f", report.MaxGrossWeight                          ),
						"number_of_engines":  fmt.Sprintf("%.1f", report.NumberOfEngines                         ),
						"overspeed_warning":  fmt.Sprintf("%.1f", report.OverspeedWarning                        ),
						"plane_alt_above_ground":  fmt.Sprintf("%.1f", report.PlaneAltAboveGround                     ),
						"plane_heading_degrees_true":  fmt.Sprintf("%.1f", report.PlaneHeadingDegreesTrue                 ),
						"plane_latitude":  fmt.Sprintf("%.1f", report.PlaneLatitude                           ),
						"plane_longitude":  fmt.Sprintf("%.1f", report.PlaneLongitude                          ),
						"rudder_position":  fmt.Sprintf("%.1f", report.RudderPosition                          ),
						"rudder_trim":  fmt.Sprintf("%.1f", report.RudderTrim                              ),
						"rudder_trim_pct":  fmt.Sprintf("%.1f", report.RudderTrimPct                           ),
						"sim_on_ground":  fmt.Sprintf("%.1f", report.SimOnGround                             ),
						"spoilers_handle_position":  fmt.Sprintf("%.1f", report.SpoilersHandlePosition                  ),
						"stall_warning":  fmt.Sprintf("%.1f", report.StallWarning                            ),

						"title":  fmt.Sprintf("%s", report.Title                            ),
						"total_weight":  fmt.Sprintf("%.1f", report.TotalWeight                             ),
						"trailing_edge_flaps_left_angle":  fmt.Sprintf("%.1f", report.TrailingEdgeFlapsLeftAngle              ),
						"trailing_edge_flaps_right_angle":  fmt.Sprintf("%.1f", report.TrailingEdgeFlapsRightAngle             ),
						"unlimited_fuel":  fmt.Sprintf("%t", report.UnlimitedFuel                           ),
						"variometer_rate":  fmt.Sprintf("%.1f", report.VariometerRate                          ),
						"vertical_speed":  fmt.Sprintf("%.1f", report.VerticalSpeed                           ),

					})

					// serial
					//altitude_tmp := []byte( fmt.Sprintf("%.0f\n", report.Altitude) )
					//airspeed_tmp := []byte( fmt.Sprintf("%.0f\n", report.Airspeed) )
					//heading_tmp := []byte( fmt.Sprintf("%.0ff\n", report.Heading) )

					//n, err = s1.Write(altitude_tmp)
					//n, err = s1.Write(airspeed_tmp)
					//n, err = s1.Write(heading_tmp)
					//s1.Write(heading_tmp)
					//if err != nil {
						//fmt.Println("err serialwrite-problem:", err)
					//}

					//fmt.Println("n serialwrite:", n)

					fmt.Println("AI TRAFFIC STATE= ", report.AiTrafficState)
					fmt.Println("AI TRAFFIC ASSIGNED RUNWAY= ", report.AiTrafficAssignedRunway)
					fmt.Println("AI TRAFFIC TOAIRPORT= ", report.AiTrafficToairport)
					fmt.Println("AUTOPILOT AVAILABLE= ", report.AutopilotAvailable)
					fmt.Println("AUTOPILOT MASTER= ", report.AutopilotMaster)
					fmt.Println("AUTOPILOT FLIGHT DIRECTOR ACTIVE= ", report.AutopilotFlightDirectorActive)
					fmt.Println("AVATAR MODE IS ATTACHED= ", report.AvatarModeIsAttached)
					fmt.Println("AVIONICS MASTER SWITCH= ", report.AvionicsMasterSwitch)
					fmt.Println("BRAKE PARKING POSITION= ", report.BrakeParkingPosition)
					fmt.Println("BRAKE PARKING INDICATOR= ", report.BrakeParkingIndicator)
					fmt.Println("FLAPS AVAILABLE= ", report.FlapsAvailable)
					fmt.Println("GPS GROUND SPEED= ", report.GpsGroundSpeed)
					fmt.Println("FLAPS HANDLE INDEX= ", report.FlapsHandleIndex)
					fmt.Println("FLAPS NUM HANDLE POSITIONS= ", report.FlapsNumHandlePositions)
					fmt.Println("ELEVATOR TRIM PCT= ", report.ElevatorTrimPct)
					fmt.Println("FLAPS HANDLE PERCENT= ", report.FlapsHandlePercent)
					fmt.Println("RUDDER TRIM PCT= ", report.RudderTrimPct)
					fmt.Println("ELEVATOR DEFLECTION PCT= ", report.ElevatorDeflectionPct)
					fmt.Println("RUDDER TRIM PCT= ", report.RudderTrimPct)
					fmt.Println("RUDDER POSITION= ", report.RudderPosition)
					fmt.Println("BRAKE INDICATOR= ", report.BrakeIndicator)
					fmt.Println("SPOILERS HANDLE POSITION= ", report.SpoilersHandlePosition)
					fmt.Println("TRAILING EDGE FLAPS LEFT ANGLE= ", report.TrailingEdgeFlapsLeftAngle)
					fmt.Println("TRAILING EDGE FLAPS RIGHT ANGLE= ", report.TrailingEdgeFlapsRightAngle)
					fmt.Println("RUDDER TRIM= ", report.RudderTrim)
					fmt.Println("MASTER IGNITION SWITCH= ", report.MasterIgnitionSwitch)
					fmt.Println("GENERAL ENG STARTER ACTIVE= ", report.GeneralEngStarterActive)
					fmt.Println("NUMBER OF ENGINES= ", report.NumberOfEngines)
					fmt.Println("ENG MAX RPM= ", report.EngMaxRpm)
					fmt.Println("AMBIENT TEMPERATURE= ", report.AmbientTemperature)
					fmt.Println("BAROMETER PRESSURE= ", report.BarometerPressure)
					fmt.Println("STALL WARNING= ", report.StallWarning)
					fmt.Println("OVERSPEED WARNING= ", report.OverspeedWarning)
					fmt.Println("INDICATED ALTITUDE= ", report.IndicatedAltitude)
					fmt.Println("VERTICAL SPEED= ", report.VerticalSpeed)
					fmt.Println("AIRSPEED TRUE= ", report.AirspeedTrue)
					fmt.Println("AIRSPEED INDICATED= ", report.AirspeedIndicated)
					fmt.Println("HEADING INDICATOR= ", report.HeadingIndicator)
					fmt.Println("UNLIMITED FUEL= ", report.UnlimitedFuel)
					fmt.Println("FUEL TOTAL QUANTITY= ", report.FuelTotalQuantity)
					fmt.Println("FUEL TOTAL CAPACITY= ", report.FuelTotalCapacity)
					fmt.Println("FUEL TOTAL QUANTITY WEIGHT= ", report.FuelTotalQuantityWeight)
					fmt.Println("GEAR TOTAL PCT EXTENDED= ", report.GearTotalPctExtended)
					fmt.Println("LIGHT TAXI ON= ", report.LightTaxiOn)
					fmt.Println("LIGHT STROBE ON= ", report.LightStrobeOn)
					fmt.Println("LIGHT PANEL ON= ", report.LightPanelOn)
					fmt.Println("LIGHT RECOGNITION ON= ", report.LightRecognitionOn)
					fmt.Println("LIGHT WING ON= ", report.LightWingOn)
					fmt.Println("LIGHT LOGO ON= ", report.LightLogoOn)
					fmt.Println("LIGHT CABIN ON= ", report.LightCabinOn)
					fmt.Println("LIGHT HEAD ON= ", report.LightHeadOn)
					fmt.Println("LIGHT BRAKE ON= ", report.LightBrakeOn)
					fmt.Println("LIGHT NAV ON= ", report.LightNavOn)
					fmt.Println("LIGHT BEACON ON= ", report.LightBeaconOn)
					fmt.Println("LIGHT LANDING ON= ", report.LightLandingOn)
					fmt.Println("MAGNETIC COMPASS= ", report.MagneticCompass)

					fmt.Println("VARIOMETER RATE= ", report.VariometerRate)
					fmt.Println("TOTAL WEIGHT= ", report.TotalWeight)
					fmt.Println("MAX GROSS WEIGHT= ", report.MaxGrossWeight)
					fmt.Println("EMPTY WEIGHT= ", report.EmptyWeight)
					fmt.Println("ELECTRICAL BATTERY LOAD= ", report.ElectricalBatteryLoad)
					fmt.Println("ELECTRICAL MASTER BATTERY= ", report.ElectricalMasterBattery)
					fmt.Println("ELECTRICAL MAIN BUS VOLTAGE= ", report.ElectricalMainBusVoltage)
					fmt.Println("SIM ON GROUND= ", report.SimOnGround)
					fmt.Println("PLANE ALT ABOVE GROUND= ", report.PlaneAltAboveGround)
					fmt.Println("PLANE LATITUDE= ", report.PlaneLatitude)
					fmt.Println("PLANE LONGITUDE= ", report.PlaneLongitude)
					fmt.Println("PLANE HEADING DEGREES TRUE= ", report.PlaneHeadingDegreesTrue)
					fmt.Println("GPS APPROACH AIRPORT ID= ", report.GpsApproachAirportId)
					fmt.Println("ATC MODEL= ", report.AtcModel)
					fmt.Println("ATC ID= ", report.AtcId)
					fmt.Println("ATC TYPE= ", report.AtcType)
					fmt.Println("ATC FLIGHT NUMBER= ", report.AtcFlightNumber)
					fmt.Println("TITLE= ", report.Title)


				case s.DefineMap["TrafficReport"]:
					trafficReport = (*TrafficReport)(ppData)
					fmt.Printf("TRAFFIC REPORT: %s\n", trafficReport.Inspect())
				}

			default:
				fmt.Println("recvInfo.ID unknown", recvInfo.ID)
			}

		case <-exitSignal:
			fmt.Println("exiting..")
			if err = s.Close(); err != nil {
				panic(err)
			}
			os.Exit(0)

		case _ = <-ws.NewConnection:
			// drain and skip

		case m := <-ws.ReceiveMessages:
			handleClientMessage(m, s)
		}
	}
}

func handleClientMessage(m websockets.ReceiveMessage, s *simconnect.SimConnect) {
	var pkt map[string]interface{}
	if err := json.Unmarshal(m.Message, &pkt); err != nil {
		fmt.Println("invalid websocket packet", err)
	} else {
		pktType, ok := pkt["type"].(string)
		if !ok {
			fmt.Println("invalid websocket packet", pkt)
			return
		}
		switch pktType {
		case "teleport":
			if disableTeleport {
				fmt.Println("teleport disabled", pkt)
				return
			}

			// validate user input
			lat, ok := pkt["lat"].(float64)
			if !ok {
				fmt.Println("invalid websocket packet", pkt)
				return
			}
			lng, ok := pkt["lng"].(float64)
			if !ok {
				fmt.Println("invalid websocket packet", pkt)
				return
			}
			altitude, ok := pkt["altitude"].(float64)
			if !ok {
				fmt.Println("invalid websocket packet", pkt)
				return
			}

			// teleport
			r := &TeleportRequest{Latitude: lat, Longitude: lng, Altitude: altitude}
			r.SetData(s)
		}
	}
}
