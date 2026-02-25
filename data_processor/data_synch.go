package data_processor

import (
	"main.go/config"

	"main.go/internal/models"
)

/*
// SynchronizedData представляет синхронизированные данные
type SynchronizedData struct {
	Timestamp time.Time
	// IMU данные
	AccelX, AccelY, AccelZ float64
	GyroX, GyroY, GyroZ    float64
	// GNSS данные (если есть)
	HasGNSS                       bool
	Latitude, Longitude, Altitude float64
	Speed                         float64
	Heading                       float64
}
*/

// ReadGNSSDataCSV_1 синхронизирует данные акселерометра, гироскопа и GNSS
func ReadGNSSDataCSV_1(accData []models.ACCData,
	gyroData []models.GYROData,
	gnssData []models.GNSSData,
	cfg *config.Config,
) ([]models.SynchronizedData, error) {

	// Проверяем, что данные IMU имеют одинаковую длину (частоту)
	if len(accData) != len(gyroData) {
		// В реальном приложении здесь нужна более сложная логика синхронизации
		// Для простоты берем минимальную длину
		minLen := len(accData)
		if len(gyroData) < minLen {
			minLen = len(gyroData)
		}
		accData = accData[:minLen]
		gyroData = gyroData[:minLen]
	}

	var syncedData []models.SynchronizedData
	gnssIndex := 0
	// Синхронизируем данные по временным меткам
	for i := 0; i < len(accData); i++ {
		acc := accData[i]
		gyro := gyroData[i]

		// Проверяем, что временные метки IMU данных близки (в пределах допустимого отклонения)
		timeDiff := acc.Timestamp.Sub(gyro.Timestamp)
		if timeDiff.Abs() > cfg.Sensors.SyncThreshold {
			// Пропускаем несинхронные данные или используем интерполяцию
			continue
		}

		// Используем среднее время между акселерометром и гироскопом
		avgTime := acc.Timestamp.Add(timeDiff / 2)

		data := models.SynchronizedData{
			Timestamp: avgTime,
			AccelX:    acc.AccelX,
			AccelY:    acc.AccelY,
			AccelZ:    acc.AccelZ,
			GyroX:     gyro.GyroX,
			GyroY:     gyro.GyroY,
			GyroZ:     gyro.GyroZ,
			HasGNSS:   false,
		}

		// Ищем ближайшие GNSS данные
		for gnssIndex < len(gnssData) {
			gnssTime := gnssData[gnssIndex].Timestamp
			timeDiffGNSS := avgTime.Sub(gnssTime).Abs()

			// Если GNSS данные в пределах окна синхронизации
			if timeDiffGNSS <= cfg.Sensors.GNSS.SyncWindow {
				data.HasGNSS = true
				data.Latitude = gnssData[gnssIndex].Latitude
				data.Longitude = gnssData[gnssIndex].Longitude
				data.Altitude = gnssData[gnssIndex].Altitude
				data.Speed = gnssData[gnssIndex].Speed

				// Если время точно совпало, переходим к следующему GNSS отсчету
				if timeDiffGNSS == 0 {
					gnssIndex++
				}
				break

			} else if gnssTime.After(avgTime) {
				// GNSS данные в будущем - выходим из цикла
				break
			}

			// Переходим к следующему GNSS отсчету
			gnssIndex++
		}

		syncedData = append(syncedData, data)

	}

	return syncedData, nil
}
