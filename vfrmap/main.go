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
	"github.com/MarphXL/msfs2020-go/vfrmap/serial"
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
		AutopilotAvailable                      bool       `name:"AUTOPILOT AVAILABLE" unit:"bool"` // Autopilot:Available flag
		AutopilotFlightDirectorActive           bool       `name:"AUTOPILOT FLIGHT DIRECTOR ACTIVE" unit:"bool"` // Autopilot:Flight director active
		AutopilotMaster                         bool       `name:"AUTOPILOT MASTER" unit:"bool"` // Autopilot:On/off flag
		AvatarModeIsAttached                    bool       `name:"AVATAR MODE IS ATTACHED" unit:"bool"` // Avatar Mode:Sets/gets the attachment state of the user avatar.
		AvionicsMasterSwitch                    bool       `name:"AVIONICS MASTER SWITCH" unit:"bool"` // Avionics:Avionics switch state
		BarometerPressure                       float64    `name:"BAROMETER PRESSURE" unit:"millibars"` // Environment:Barometric pressure
		BrakeIndicator                          float64    `name:"BRAKE INDICATOR" unit:"position"` // Avionics:Brake on indication [0.0: Off, 1.0: Full Brakes]
		BrakeParkingIndicator                   bool       `name:"BRAKE PARKING INDICATOR" unit:"bool"` // Avionics:Parking brake indicator
		BrakeParkingPosition                    bool       `name:"BRAKE PARKING POSITION" unit:"bool"` // Avionics:Parking brake on [0: Off, 1.0: Full Parking Brake]
		ElectricalBatteryLoad                   float64    `name:"ELECTRICAL BATTERY LOAD" unit:"ampere"` // Miscellaneous Systems:Battery load
		ElectricalMainBusVoltage                float64    `name:"ELECTRICAL MAIN BUS VOLTAGE" unit:"volts"` // Miscellaneous Systems:Main bus voltage
		ElectricalMasterBattery                 bool       `name:"ELECTRICAL MASTER BATTERY" unit:"bool"` // Miscellaneous Systems:Battery switch position
		ElevatorDeflectionPct                   float64    `name:"ELEVATOR DEFLECTION PCT" unit:"percent over 100"` // Avionics:Percent deflection
		ElevatorTrimPct                         float64    `name:"ELEVATOR TRIM PCT" unit:"percent over 100"` // Avionics:Percent elevator trim
		EmptyWeight                             float64    `name:"EMPTY WEIGHT" unit:"pounds"` // Miscellaneous:Empty weight of the aircraft
		EngMaxRpm                               float64    `name:"ENG MAX RPM" unit:"rpm"` // Engine:Maximum rpm
		FlapsAvailable                          bool       `name:"FLAPS AVAILABLE" unit:"bool"` // Avionics:True if flaps available
		FlapsHandleIndex                        float64    `name:"FLAPS HANDLE INDEX" unit:"number"` // Avionics:Index of current flap position
		FlapsHandlePercent                      float64    `name:"FLAPS HANDLE PERCENT" unit:"percent over 100"` // Avionics:Percent flap handle extended
		FlapsNumHandlePositions                 float64    `name:"FLAPS NUM HANDLE POSITIONS" unit:"number"` // Avionics:Number of flap positions
		FuelTotalCapacity                       float64    `name:"FUEL TOTAL CAPACITY" unit:"gallons"` // Fuel:Total capacity of the aircraft
		FuelTotalQuantity                       float64    `name:"FUEL TOTAL QUANTITY" unit:"gallons"` // Fuel:Current quantity in volume
		FuelTotalQuantityWeight                 float64    `name:"FUEL TOTAL QUANTITY WEIGHT" unit:"pounds"` // Fuel:Current total fuel weight of the aircraft
		GearTotalPctExtended                    float64    `name:"GEAR TOTAL PCT EXTENDED" unit:"percent"` // Landing Gear:Percent total gear extended
		GeneralEngStarterActive                 bool       `name:"GENERAL ENG STARTER ACTIVE" unit:"bool"` // Engine:True if engine starter is active
		GpsApproachAirportId                    [256]byte  `name:"GPS APPROACH AIRPORT ID" unit:"string"` // String:ID of airport
		GpsGroundSpeed                          float64    `name:"GPS GROUND SPEED" unit:"meters per second"` // Avionics:Current ground speed
		HeadingIndicator                        float64    `name:"HEADING INDICATOR" unit:"radians"` // Flight Instrumentation:Heading indicator (directional gyro) indication
		IndicatedAltitude                       float64    `name:"INDICATED ALTITUDE" unit:"feet"` // Flight Instrumentation:Altimeter indication
		LightBeaconOn                           bool       `name:"LIGHT BEACON ON" unit:"bool"` // Lights:Return true if the light is on.
		LightBrakeOn                            bool       `name:"LIGHT BRAKE ON" unit:"bool"` // Lights:Return true if the light is on.
		LightCabinOn                            bool       `name:"LIGHT CABIN ON" unit:"bool"` // Lights:Return true if the light is on.
		LightHeadOn                             bool       `name:"LIGHT HEAD ON" unit:"bool"` // Lights:Return true if the light is on.
		LightLandingOn                          bool       `name:"LIGHT LANDING ON" unit:"bool"` // Lights:Return true if the light is on.
		LightLogoOn                             bool       `name:"LIGHT LOGO ON" unit:"bool"` // Lights:Return true if the light is on.
		LightNavOn                              bool       `name:"LIGHT NAV ON" unit:"bool"` // Lights:Return true if the light is on.
		LightPanelOn                            bool       `name:"LIGHT PANEL ON" unit:"bool"` // Lights:Return true if the light is on.
		LightRecognitionOn                      bool       `name:"LIGHT RECOGNITION ON" unit:"bool"` // Lights:Return true if the light is on.
		LightStrobeOn                           bool       `name:"LIGHT STROBE ON" unit:"bool"` // Lights:Return true if the light is on.
		LightTaxiOn                             bool       `name:"LIGHT TAXI ON" unit:"bool"` // Lights:Return true if the light is on.
		LightWingOn                             bool       `name:"LIGHT WING ON" unit:"bool"` // Lights:Return true if the light is on.
		MagneticCompass                         float64    `name:"MAGNETIC COMPASS" unit:"degrees"` // Miscellaneous:Compass reading
		MasterIgnitionSwitch                    bool       `name:"MASTER IGNITION SWITCH" unit:"bool"` // Engine:Aircraft master ignition switch (grounds all engines magnetos)
		MaxGrossWeight                          float64    `name:"MAX GROSS WEIGHT" unit:"pounds"` // Miscellaneous:Maximum gross weight of the aircraft
		NumberOfEngines                         float64    `name:"NUMBER OF ENGINES" unit:"number"` // Engine:Number of engines (minimum 0, maximum 4)
		OverspeedWarning                        bool       `name:"OVERSPEED WARNING" unit:"bool"` // Flight Instrumentation:Overspeed warning state
		PlaneAltAboveGround                     float64    `name:"PLANE ALT ABOVE GROUND" unit:"feet"` // Position and Speed:Altitude above the surface
		PlaneHeadingDegreesTrue                 float64    `name:"PLANE HEADING DEGREES TRUE" unit:"radians"` // Position and Speed:Heading relative to true north, although the name m
		PlaneLatitude                           float64    `name:"PLANE LATITUDE" unit:"radians"` // Position and Speed:Latitude of aircraft, North is positive, South nega
		PlaneLongitude                          float64    `name:"PLANE LONGITUDE" unit:"radians"` // Position and Speed:Longitude of aircraft, East is positive, West negat
		RudderPosition                          float64    `name:"RUDDER POSITION" unit:"position"` // Avionics:Rudder input deflection [-1.0: Full Left, 1.0: Full Right]
		RudderTrim                              float64    `name:"RUDDER TRIM" unit:"radians"` // Avionics:Angle deflection
		RudderTrimPct                           float64    `name:"RUDDER TRIM PCT" unit:"percent over 100"` // Avionics:Percent deflection
		RudderTrimPct                           float64    `name:"RUDDER TRIM PCT" unit:"percent over 100"` // Avionics:The trim position of the rudder. Zero is no trim.
		SimOnGround                             bool       `name:"SIM ON GROUND" unit:"bool"` // Position and Speed:On ground flag (To set, passing a non-zero argument
		SpoilersHandlePosition                  float64    `name:"SPOILERS HANDLE POSITION" unit:"position"` // Avionics:Spoiler handle position [0: Retracted, 1.0: Fully Extended]
		StallWarning                            bool       `name:"STALL WARNING" unit:"bool"` // Flight Instrumentation:Stall warning state
		SurfaceCondition                        float64    `name:""SURFACE CONDITION"" unit:""enum""` // Miscellaneous:One of:0: Normal 1: Wet 2: Icy 3: Snow"
		Title                                   [256]byte  `name:"TITLE" unit:"variable length string"` // String:title from aircraft.cfg
		TotalWeight                             float64    `name:"TOTAL WEIGHT" unit:"pounds"` // Miscellaneous:Total weight of the aircraft
		TrailingEdgeFlapsLeftAngle              float64    `name:"TRAILING EDGE FLAPS LEFT ANGLE" unit:"radians"` // Avionics:Angle left trailing edge flap extended. Use TRAILING EDGE FLA
		TrailingEdgeFlapsRightAngle             float64    `name:"TRAILING EDGE FLAPS RIGHT ANGLE" unit:"radians"` // Avionics:Angle right trailing edge flap extended. Use TRAILING EDGE FL
		TransponderCode_i                       float64    `name:"TRANSPONDER CODE:index" unit:"bco16"` // Avionics:4-digit code
		UnlimitedFuel                           bool       `name:"UNLIMITED FUEL" unit:"bool"` // Fuel:Unlimited fuel flag
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
	c := &serial.Config{Name: "COM31", Baud: 9600}
        s1, err := serial.OpenPort(c)
        if err != nil {
                panic(err)
        }
	fmt.Println("serial connected to Arduino!")

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
						"type":           "plane",
						"latitude":       report.Latitude,
						"longitude":      report.Longitude,
						"altitude":       fmt.Sprintf("%.0f", report.Altitude),
						"heading":        int(report.Heading),
						"airspeed":       fmt.Sprintf("%.0f", report.Airspeed),
						"airspeed_true":  fmt.Sprintf("%.0f", report.AirspeedTrue),
						"vertical_speed": fmt.Sprintf("%.0f", report.VerticalSpeed),
						"flaps":          fmt.Sprintf("%.0f", report.Flaps),
						"trim":           fmt.Sprintf("%.1f", report.Trim),
						"rudder_trim":    fmt.Sprintf("%.1f", report.RudderTrim),

						"fuel_total_quantity": 	fmt.Sprintf("%.1f", report.FuelQuantity),
						"fuel_total_capacity":		fmt.Sprintf("%.1f", report.FuelCapacity),
						"master_ignition_switch":	fmt.Sprintf("%.1f", report.MasterIgnitionSwitch),

					})

					// serial
					altitude_tmp := []byte( fmt.Sprintf("%.0f\n", report.Altitude) )
					airspeed_tmp := []byte( fmt.Sprintf("%.0f\n", report.Airspeed) )
					heading_tmp := []byte( fmt.Sprintf("%.0ff\n", report.Heading) )

					n, err = s1.Write(altitude_tmp)
					n, err = s1.Write(airspeed_tmp)
					n, err = s1.Write(heading_tmp)

					if err != nil {
						fmt.Println("err serialwrite-problem:", err)
					}

					fmt.Println("n serialwrite:", n)

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
