package models

import (
	"time"
)

// AccelerometerData представляет данные акселерометра
type ACCData struct {
	Timestamp time.Time // Временная метка
	AccelX    float64   // Ускорение по X (м/с²)
	AccelY    float64   // Ускорение по Y (м/с²)
	AccelZ    float64   // Ускорение по Z (м/с²)
}

// AccelerometerData представляет данные акселерометра
type GYROData struct {
	Timestamp time.Time // Временная метка
	GyroX     float64   // Угловая скорость по X (рад/с)
	GyroY     float64   // Угловая скорость по Y (рад/с)
	GyroZ     float64   // Угловая скорость по Z (рад/с)
}

// GNSSData представляет данные GNSS/GPS
type GNSSData struct {
	Timestamp time.Time // Временная метка
	Latitude  float64   // Широта (градусы)
	Longitude float64   // Долгота (градусы)
	Altitude  float64   // Высота (метры)
	Speed     float64   // Скорость (м/с)
	Heading   float64   // Направление (градусы)
}

// EstimatedState представляет оцененное состояние
type EstimatedState struct {
	Timestamp time.Time // Временная метка
	PositionX float64   // Позиция X (метры)
	PositionY float64   // Позиция Y (метры)
	PositionZ float64   // Позиция Z (метры)

	QuaternionW float64 // Кватернион W
	QuaternionX float64 // Кватернион X
	QuaternionY float64 // Кватернион Y
	QuaternionZ float64 // Кватернион Z

	CovarianceXX float64 // Дисперсия позиции X
	CovarianceYY float64 // Дисперсия позиции Y
	CovarianceZZ float64 // Дисперсия позиции Y

	CovarianceQwQw float64 // Дисперсия кватерниона W
	CovarianceQxQx float64 // Дисперсия кватерниона X
	CovarianceQyQy float64 // Дисперсия кватерниона Y
	CovarianceQzQz float64 // Дисперсия кватерниона Z

}

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
}
