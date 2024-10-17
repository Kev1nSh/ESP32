package main

import (
	"bufio"
	"fmt"
	"net"
	//"regexp"
	"strconv"
	"strings"
	"sync"
)


var mu sync.Mutex
var latestSensorValue int
var macAddress []string

func main() {

	// Create a listener on TCP port 8080
	listener, err := net.Listen("tcp", ":1256")
	if err != nil {
		fmt.Println("Error starting server", err.Error())
		return
	}

	//Close the listener when the application closes
	defer listener.Close()
	fmt.Println("Server started on :1256, waiting for connections...")

	for {

		//Accept connection from client
		conn, err := listener.Accept()
		if err != nil {
			fmt.Println("Error accepting connection", err.Error())
			return
		}

		fmt.Println("Accepted connection from", conn.RemoteAddr().String())

		//Handle the connection in a new goroutine
		go handleConnection(conn)
	}
}
/*
func isMacAddress(input string) bool {
	macRegex := `^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$`
	re := regexp.MustCompile(macRegex)
	return re.MatchString(input)
}
*/
func handleMACAddress(conn net.Conn, message string) {

	fmt.Println("Received MAC address:", message)

	for _, mac := range macAddress {
		if mac == message {
			fmt.Println("MAC address already exists")
			conn.Write([]byte("MAC address already exists\n"))
			return
		}
	}

	macAddress = append(macAddress, message)
	fmt.Println("MAC address stored", message)
	conn.Write([]byte("MAC address stored\n"))
}
/*
func handleSensorValue(conn net.Conn, message string) {

	re := regexp.MustCompile(`\d+`) // Find all digits in the message
	match := re.FindString(message)

	if match != "" {
		sensorValue, err := strconv.Atoi(match)
		mu.Lock()
		latestSensorValue = sensorValue
		mu.Unlock()
		if err == nil {
			if sensorValue >= 50 {
				conn.Write([]byte("TURN_ON_LED"))
			} else {
				conn.Write([]byte("TURN_OFF_LED"))
			}
		} else {
			fmt.Println("Error converting sensor value", err.Error())
		}
	} else {
		fmt.Println("No integer found in message:", message)
	}
}

func handleConnection(conn net.Conn) {

	defer conn.Close()

	buf := make([]byte, 1024)
	for {
		n, err := conn.Read(buf)
		if err != nil {
			fmt.Println("Error reading:", err.Error())
			return
		}

		message := string(buf[:n])
		message = strings.TrimSpace(message)
		fmt.Println("Received message:", message)

		if isMacAddress(message) {
			handleMACAddress(conn, message)
			fmt.Printf("MAC address: %s\n", message)
		} else {
			handleSensorValue(conn, message)
		}
	}
}
*/

func handleConnection(conn net.Conn) {

	//Close the connection when the function exits
	defer conn.Close()

	//Create a new reader for the connection
	reader := bufio.NewReader(conn)

	for {

		//Read the incoming message
		message, err := reader.ReadString('\n')
		if err != nil {
			fmt.Println("Error reading:", err.Error())
			return
		}

		//Trim the message of whitespace
		message = strings.TrimSpace(message)
		fmt.Println("Received message:", message)

		//Check if the message is a MAC address
		if strings.HasPrefix(message, "MAC:") {
			handleMACAddress(conn, strings.TrimPrefix((message), "MAC:"))
		} else if strings.HasPrefix((message), "Sensor value:"){
			parts := strings.Split(message, ":")
			if len(parts) == 2 {
				sensorValue, err := strconv.Atoi(strings.TrimSpace(parts[1]))
				if err == nil {
					if sensorValue >= 50 {
						conn.Write([]byte("TURN_ON_LED"))
					} else {
						conn.Write([]byte("TURN_OFF_LED"))
					}
				} else {
					fmt.Println("Error converting sensor value", err.Error())
				}
			} else {
				fmt.Println("Unexpected message format:", message)
			}
		} else {
			fmt.Println("Unknown message type:", message)
		}
	}
}

