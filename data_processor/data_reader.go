package data_processor

import (
	"encoding/csv"
	"os"
	"strconv"
	"time"

	"main.go/internal/models"
)

// ReadAccelerometerCSV читает данные акселерометра из CSV
func ReadAccelerometerCSV(filename string) ([]models.ACCData, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll()
	if err != nil {
		return nil, err
	}

	var data []models.ACCData
	time_ := 0

	// Пропускаем заголовок если есть
	startIdx := 0
	if len(records) > 0 {
		// Проверяем, является ли первая строка заголовком
		if _, err := strconv.ParseFloat(records[0][0], 64); err != nil {
			startIdx = 1
		}
	}

	for i := startIdx; i < len(records); i++ {
		record := records[i]

		if len(record) < 3 {
			continue
		}

		// Предполагаем формат: timestamp, accel_x, accel_y, accel_z
		/*timestamp, err := strconv.ParseInt(record[0], 10, 64)
		if err != nil {
			continue
		}
		*/

		timestamp := time_

		time_ = time_ + 10

		accelX, _ := strconv.ParseFloat(record[1], 64)
		accelY, _ := strconv.ParseFloat(record[2], 64)
		accelZ, _ := strconv.ParseFloat(record[3], 64)

		data = append(data, models.ACCData{
			//Timestamp: time.Unix(0, timestamp*int64(time.Millisecond)),
			Timestamp: time.Unix(0, int64(timestamp)*int64(time.Millisecond)),
			AccelX:    accelX,
			AccelY:    accelY,
			AccelZ:    accelZ,
		})
	}

	return data, nil
}

// ReadGyroCSV читает данные акселерометра из CSV
func ReadGyroCSV(filename string) ([]models.GYROData, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll()
	if err != nil {
		return nil, err
	}

	var data []models.GYROData
	time_ := 0

	// Пропускаем заголовок если есть
	startIdx := 0
	if len(records) > 0 {
		// Проверяем, является ли первая строка заголовком
		if _, err := strconv.ParseFloat(records[0][0], 64); err != nil {
			startIdx = 1
		}
	}

	for i := startIdx; i < len(records); i++ {
		record := records[i]

		if len(record) < 3 {
			continue
		}

		/*
			// Предполагаем формат: timestamp, gyro_x, gyro_y, gyro_z
			timestamp, err := strconv.ParseInt(record[0], 10, 64)
			if err != nil {
				continue
			}
		*/

		timestamp := time_

		time_ = time_ + 10

		gyroX, _ := strconv.ParseFloat(record[1], 64)
		gyroY, _ := strconv.ParseFloat(record[2], 64)
		gyroZ, _ := strconv.ParseFloat(record[3], 64)

		data = append(data, models.GYROData{
			Timestamp: time.Unix(0, int64(timestamp)*int64(time.Millisecond)),
			GyroX:     gyroX,
			GyroY:     gyroY,
			GyroZ:     gyroZ,
		})
	}

	return data, nil
}

// ReadGNSSDataCSV читает данные GNSS из CSV
func ReadGNSSDataCSV(filename string) ([]models.GNSSData, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	reader := csv.NewReader(file)
	records, err := reader.ReadAll()
	if err != nil {
		return nil, err
	}

	var data []models.GNSSData
	time_ := 0

	startIdx := 0
	if len(records) > 0 {
		if _, err := strconv.ParseFloat(records[0][0], 64); err != nil {
			startIdx = 1
		}
	}

	for i := startIdx; i < len(records); i++ {
		record := records[i]

		if len(record) < 2 {
			continue
		}

		/*
			// Формат: timestamp, latitude, longitude, altitude, speed, heading
			timestamp, err := strconv.ParseInt(record[0], 10, 64)
			if err != nil {
				continue
			}
		*/

		timestamp := time_

		time_ = time_ + 1000

		lat, _ := strconv.ParseFloat(record[10], 64)
		lon, _ := strconv.ParseFloat(record[9], 64)
		alt, _ := strconv.ParseFloat(record[8], 64)

		speed, _ := strconv.ParseFloat(record[6], 64)

		dataPoint := models.GNSSData{
			Timestamp: time.Unix(0, int64(timestamp)*int64(time.Millisecond)),
			Latitude:  lat,
			Longitude: lon,
			Altitude:  alt,
			Speed:     speed,
		}

		data = append(data, dataPoint)
	}

	return data, nil
}
