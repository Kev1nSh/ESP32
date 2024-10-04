package main

import (
	"bufio"
	"fmt"
	"net"
	"strconv"
	"strings"
)

func main() {

	// Create a listener on TCP port 8080
	listener, err := net.Listen("tcp", "localhost:8080")
	if err != nil {
		fmt.Println("Error starting server", err.Error())
		return
	}

	//Close the listener when the application closes
	defer listener.Close()

	fmt.Println("Server started on :8080, waiting for connections...")

	for {

		//Accept connection from client
		conn, err := listener.Accept()
		if err != nil {
			fmt.Println("Error accepting connection", err.Error())
			return
		}

		//Handle the connection in a new goroutine
		go handleConnection(conn)

	}

}

func handleConnection(conn net.Conn) {

	//Close the connection when the function exits
	defer conn.Close()

	//Create a new reader for the connection
	reader := bufio.NewReader(conn)

	for {

		//Read the incoming message
		message, err := reader.ReadString('\n')
		if err != nil {
			fmt.Println("Error reading", err.Error())
			return
		}

		//Trim the message of whitespace
		message = strings.Trim(message, "\n")
		fmt.Println("Received message:", message)

		parts := strings.Split(message, ":")
		if len(parts) == 2 && strings.TrimSpace(parts[0]) == "Sensor value" {
			sensorValue, err := strconv.Atoi(strings.TrimSpace(parts[1]))
			if err == nil {
				if sensorValue > 5 {
					conn.Write([]byte("TURN_ON_LED\n"))
				} else {
					conn.Write([]byte("TURN_OFF_LED\n"))
				}

			}

		}

		//Convert the message to an integer

		/*
			brightness, err := strconv.Atoi(message)

			if err != nil {
				fmt.Println("Invalid brightness value", err.Error())
				continue
			}
		*/

		//brightness := 4 //Fake brightness value

		//Check if the brightness is greater than or equal to 3

	}

}
