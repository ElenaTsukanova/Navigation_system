package fuzzer

import (
	"math"

	"main.go/config"
)

// Параметры эллипсоида WGS 84
const (
	a  = 6378137.0         // большая полуось (м)
	f  = 1 / 298.257223563 // сжатие
	e2 = 2*f - f*f         // квадрат эксцентриситета
)

// DegreesToRadians преобразует градусы в радианы
func DegreesToRadians(deg float64) float64 {
	return deg * math.Pi / 180.0
}

// RadiansToDegrees преобразует радианы в градусы
func RadiansToDegrees(rad float64) float64 {
	return rad * 180.0 / math.Pi
}

// GeodeticToECEF преобразует геодезические координаты в ECEF
func GeodeticToECEF(lat, lon, alt float64) (x, y, z float64) {
	sinLat := math.Sin(lat)
	cosLat := math.Cos(lat)
	sinLon := math.Sin(lon)
	cosLon := math.Cos(lon)

	// Радиус кривизны первого вертикала
	N := a / math.Sqrt(1-e2*sinLat*sinLat)

	// Координаты в ECEF
	x = (N + alt) * cosLat * cosLon
	y = (N + alt) * cosLat * sinLon
	z = (N*(1-e2) + alt) * sinLat

	return x, y, z
}

// GeodeticToENU конвертирует широту/долготу в метры относительно начальной точки (ENU)
func GeodeticToENU(lat, lon, alt float64, cfg *config.Config) (float64, float64, float64) {
	// 0. Перевод пришедших широты и долготы из градусов в радианы
	targetLat := DegreesToRadians(lat)
	targetLon := DegreesToRadians(lon)
	targeAlt := alt

	// Предполагаем, что начальная точка хранится в конфиге
	refLat := DegreesToRadians(cfg.Sensors.GNSS.ReferenceLatitude)
	refLon := DegreesToRadians(cfg.Sensors.GNSS.ReferenceLongitude)
	refAlt := cfg.Sensors.GNSS.ReferenceAltitude

	// 1. Преобразуем обе точки в ECEF
	x1, y1, z1 := GeodeticToECEF(targetLat, targetLon, targeAlt)
	x0, y0, z0 := GeodeticToECEF(refLat, refLon, refAlt)

	// 2. Вектор сдвига в ECEF
	dx := x1 - x0
	dy := y1 - y0
	dz := z1 - z0

	// 3. Тригонометрические функции для опорной точки
	sinLatRef := math.Sin(refLat)
	cosLatRef := math.Cos(refLat)
	sinLonRef := math.Sin(refLon)
	cosLonRef := math.Cos(refLon)

	// 4. Вычисляем координаты ENU
	east := -sinLonRef*dx + cosLonRef*dy
	north := -sinLatRef*cosLonRef*dx - sinLatRef*sinLonRef*dy + cosLatRef*dz
	up := cosLatRef*cosLonRef*dx + cosLatRef*sinLonRef*dy + sinLatRef*dz

	return east, north, up

}

// ECEFToGeodetic преобразует ECEF в геодезические координаты (итеративный метод)
func ECEFToGeodetic(x, y, z float64) (lat, lon, alt float64) {
	// Долгота
	lon = math.Atan2(y, x)

	// Итеративное вычисление широты
	p := math.Sqrt(x*x + y*y)
	lat = math.Atan2(z, p*(1-e2)) // Начальное приближение
	var prevLat float64

	// Итерация до сходимости
	for i := 0; i < 10; i++ {
		sinLat := math.Sin(lat)
		N := a / math.Sqrt(1-e2*sinLat*sinLat)
		alt := p/math.Cos(lat) - N
		prevLat = lat
		lat = math.Atan2(z, p*(1-e2*N/(N+alt)))

		if math.Abs(lat-prevLat) < 1e-12 {
			break
		}
	}

	// Высота
	sinLat := math.Sin(lat)
	N := a / math.Sqrt(1-e2*sinLat*sinLat)
	alt = p/math.Cos(lat) - N

	// Перевод геодезических координат из радиан в градусы
	lat = RadiansToDegrees(lat)
	lon = RadiansToDegrees(lon)

	return lat, lon, alt
}

// ENUToGeodetic обратное преобразование: ENU → геодезические координаты
func ENUToGeodetic(east, north, up float64, cfg *config.Config) (float64, float64, float64) {

	refLat := DegreesToRadians(cfg.Sensors.GNSS.ReferenceLatitude)
	refLon := DegreesToRadians(cfg.Sensors.GNSS.ReferenceLongitude)
	refAlt := cfg.Sensors.GNSS.ReferenceAltitude

	// Тригонометрические функции для опорной точки
	sinLatRef := math.Sin(refLat)
	cosLatRef := math.Cos(refLat)
	sinLonRef := math.Sin(refLon)
	cosLonRef := math.Cos(refLon)

	// Матрица поворота (транспонированная)
	dx := -sinLonRef*east - sinLatRef*cosLonRef*north + cosLatRef*cosLonRef*up
	dy := cosLonRef*east - sinLatRef*sinLonRef*north + cosLatRef*sinLonRef*up
	dz := 0*east + cosLatRef*north + sinLatRef*up

	// Координаты ECEF для цели
	x0, y0, z0 := GeodeticToECEF(refLat, refLon, refAlt)
	x1 := x0 + dx
	y1 := y0 + dy
	z1 := z0 + dz

	// Обратное преобразование ECEF → геодезические
	return ECEFToGeodetic(x1, y1, z1)
}
