package fuzzer

import (
	"math"
)

// AccTransformation преобразует данные акселеромтера в систему ENU (на вход поступают данные под 45° по направлению движения автом - X слева, Y справа, Z вверх)
func IMUTransformation(X, Y, Z float64) (float64, float64, float64) {
	// cos(45°) = sin(45°) = 0.70710678118
	cos45 := 1 / math.Sqrt(2)
	sin45 := 1 / math.Sqrt(2)

	// Поворот против часовой стрелки с инвертированием оси X
	Xcorr := -X*cos45 + Y*sin45
	Ycorr := X*sin45 + Y*cos45

	// Ось Z не меняется
	Zcorr := Z

	return Xcorr, Ycorr, Zcorr
}

// AccTransformation преобразует данные акселеромтера в систему ENU (на вход поступают данные под 45° по направлению движения автом - X справа, Y слева, Z вниз)
func IMUTransformation_1(X, Y, Z float64) (float64, float64, float64) {
	// cos(45°) = sin(45°) = 0.70710678118
	cos45 := 1 / math.Sqrt(2)
	sin45 := 1 / math.Sqrt(2)

	// Поворот против часовой стрелки с инвертированием оси X
	Xcorr := X*cos45 + Y*sin45
	Ycorr := -X*sin45 + Y*cos45

	// Ось Z не меняется
	Zcorr := -Z

	return Xcorr, Ycorr, Zcorr
}

// AccTransformation преобразует данные акселеромтера в систему ENU (на вход поступают данные под 45° против направления движения авто - X слева, Y справа, Z вверх)
func IMUTransformation_2(X, Y, Z float64) (float64, float64, float64) {
	// поворот на 135* против часовой стрелки
	cos := math.Cos(DegreesToRadians(135))
	sin := math.Sin(DegreesToRadians(135))

	// Поворот против часовой стрелки с инвертированием оси X
	Xcorr := X*cos - Y*sin
	Ycorr := X*sin + Y*cos

	// Ось Z не меняется
	Zcorr := Z

	return Xcorr, Ycorr, Zcorr
}
