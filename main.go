// package Navigation_system_v_1
package main

import (
	"fmt"
	"log"

	"main.go/config"
	"main.go/internal/fuzzer"
	"main.go/internal/models"

	"main.go/data_processor"
)

func main() {
	// 1. Загрузка конфигурации
	cfg, err := config.LoadConfig("config/config.yaml")
	if err != nil {
		log.Fatal("Ошибка загрузки конфигурации:", err)
	}

	// 2. Чтение данных
	// Чтение данных датчиков
	accData, err := data_processor.ReadAccelerometerCSV("data/acc_31_07.csv")
	if err != nil {
		log.Fatal("Ошибка чтения данных акселерометра:", err)
	}

	gyroData, err := data_processor.ReadGyroCSV("data/gyro_31_07.csv")
	if err != nil {
		log.Fatal("Ошибка чтения данных акселерометра:", err)
	}

	gnssData, err := data_processor.ReadGNSSDataCSV("data/gnss_31_07.csv")
	if err != nil {
		log.Fatal("Ошибка чтения данных GNSS:", err)
	}

	fmt.Printf("\nЗагружено %d записей акселерометра\n", len(accData))
	fmt.Printf("Загружено %d записей гироскопа\n", len(gyroData))
	fmt.Printf("Загружено %d записей GNSS\n", len(gnssData))

	// 3. Синхронизируем данные всех датчиков
	syncedData, err := data_processor.ReadGNSSDataCSV_1(accData, gyroData, gnssData, cfg)

	// 4. Основной цикл обработки

	runNavigation(syncedData, cfg)

	fmt.Printf("Состояние: X=%d\n", len(accData))

}

func runNavigation(syncedData []models.SynchronizedData, cfg *config.Config) ([]models.EstimatedState, error) {

	fmt.Println("Запуск навигационной системы...")

	// 1. Создание процессора данных
	fuzzer := fuzzer.NewFuzzer(cfg)

	//2.  Обработка данных
	results, err := fuzzer.Process(syncedData)
	if err != nil {
		log.Fatal("Ошибка обработки данных:", err)
	}

	/*

		//var lastACC time.Time
		//var lastGYRO time.Time
		//var lastGNSS time.Time
			for {
				select {
				case accData := <-accChan:
					// Прогноз на основе ACC
					err := ekf.Predict(accData)
					if err != nil {
						log.Printf("Ошибка прогноза: %v", err)
						continue
					}

					lastACC = time.Now()
					// Вывод состояния каждые 0.1 секунды
					if time.Since(lastACC) > 100*time.Millisecond {
						state := filter.GetState()
						fmt.Printf("Состояние: X=%.2f, Y=%.2f, V=%.2f\n",
							state[0], state[1], state[3])
					}

				case gyroData := <-gyroChan:
					// Прогноз на основе GYRO
					err := ekf.Predict(gyroData)
					if err != nil {
						log.Printf("Ошибка прогноза: %v", err)
						continue
					}

					lastGYRO = time.Now()
					// Вывод состояния каждые 0.1 секунды
					if time.Since(lastGYRO) > 100*time.Millisecond {
						state := filter.GetState()
						fmt.Printf("Состояние: X=%.2f, Y=%.2f, V=%.2f\n",
							state[0], state[1], state[3])
					}


				case gnssData := <-gnssChan:
					// Коррекция на основе GNSS
					err := filter.Update(gnssData)
					if err != nil {
						log.Printf("Ошибка коррекции: %v", err)
						continue
					}

					lastGNSS = time.Now()
					fmt.Printf("GNSS коррекция: lat=%.6f, lon=%.6f\n",
						gnssData.Latitude, gnssData.Longitude)

					// Сохранение результатов
					filter.SaveState("output/navigation_result.csv")
				}
			}
	*/

	return results, nil
}
