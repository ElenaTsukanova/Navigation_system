package ekf

import (
	"fmt"
	"time"

	filter "github.com/milosgajdos/go-estimate"
	"github.com/milosgajdos/go-estimate/estimate"
	"github.com/milosgajdos/go-estimate/noise"
	"github.com/milosgajdos/go-estimate/sim"
	"gonum.org/v1/gonum/mat"

	"main.go/internal/models"
)

// EKFWrapper обертка для работы с EKF
type EKFWrapper struct {
	ekf      *EKF
	stateDim int
	config   *EKFConfig // Конфигурация

	initCond filter.InitCond

	lastTime     time.Time // Время последнего обновления
	lastEstimate filter.Estimate

	positionModel *models.PositionModel
}

// EKFConfig конфигурация EKF
type EKFConfig struct {
	TimeStep         float64
	InitialState     []float64
	InitialCov       []float64
	ProcessNoise     []float64
	MeasurementNoise []float64
}

// NewEKFWrapper создает новый EKF
// func NewEKFWrapper(model filter.Model, cfg *config.Config) (*EKFWrapper, error) {
func NewEKFWrapper(model filter.Model, cfg *EKFConfig) (*EKFWrapper, error) {

	// 1. Создаем вектор начального состояния
	initState := mat.NewVecDense(len(cfg.InitialState), cfg.InitialState)

	// 2. Создаем начальную ковариационную матрицу (диагональная)
	initCov := mat.NewSymDense(len(cfg.InitialCov), nil)
	for i := 0; i < len(cfg.InitialCov); i++ {
		initCov.SetSym(i, i, cfg.InitialCov[i])
	}

	//  3. Создаем структуру начальных условий
	initCond := sim.NewInitCond(initState, initCov)

	// 4. Создаем шум процесса Q
	Q := mat.NewSymDense(len(cfg.ProcessNoise), nil)
	for i := 0; i < len(cfg.ProcessNoise); i++ {
		Q.SetSym(i, i, cfg.ProcessNoise[i])
	}
	processNoise, err := noise.NewGaussian(make([]float64, len(cfg.ProcessNoise)), Q)
	if err != nil {
		return nil, fmt.Errorf("ошибка создания шума процесса: %w", err)
	}

	// 5. Создаем шум измерений R
	R := mat.NewSymDense(len(cfg.MeasurementNoise), nil)
	for i := 0; i < len(cfg.MeasurementNoise); i++ {
		R.SetSym(i, i, cfg.MeasurementNoise[i])
	}
	measNoise, err := noise.NewGaussian(make([]float64, len(cfg.MeasurementNoise)), R)
	if err != nil {
		return nil, fmt.Errorf("ошибка создания шума измерений: %w", err)
	}

	// 7. Создаем EKF с помощью New()
	ekfFilter, err := New(model, initCond, processNoise, measNoise)
	if err != nil {
		return nil, fmt.Errorf("ошибка создания EKF: %w", err)
	}

	// 8. Создаем lastEstimate из начальных условий
	lastEstimate, err := estimate.NewBaseWithCov(initState, initCov)
	if err != nil {
		return nil, fmt.Errorf("ошибка создания начального условия: %v", err)
	}

	return &EKFWrapper{
		ekf:          ekfFilter,
		stateDim:     len(cfg.InitialState),
		config:       cfg,
		lastTime:     time.Now(),
		initCond:     initCond,
		lastEstimate: lastEstimate,
	}, nil
}

// Predict выполняет предсказание
// func (w *EKFWrapper) Predict(acc *models.ACCData, gyro *models.GYROData) (*models.EstimatedState, error) {
func (w *EKFWrapper) Predict(u mat.Vector) (*models.EstimatedState, error) {

	w.lastTime = time.Now()

	// Обновляем время в модели перед предсказанием
	if model, ok := w.ekf.Model().(*models.PositionModel); ok {
		model.UpdateData()
	}

	// 1. Получаем текущее состояние из оценки
	x := w.lastEstimate.Val()

	// 2. Выполняем предсказание
	estimate, err := w.ekf.Predict(x, u)
	if err != nil {
		return nil, fmt.Errorf("ошибка предсказания: %v", err)
	}

	w.lastEstimate = estimate

	return w.estimateToState(estimate), nil
}

// Update выполняет коррекцию на основе GNSS данных
func (w *EKFWrapper) Update(z mat.Vector) (*models.EstimatedState, error) {

	// 1. Получаем текущее состояние
	x := w.lastEstimate.Val()

	// 2. Выполняем коррекцию
	estimate, err := w.ekf.Update(x, nil, z)
	if err != nil {
		return nil, fmt.Errorf("ошибка коррекции: %v", err)
	}

	w.lastEstimate = estimate

	return w.estimateToState(estimate), nil
}

// Run выполняет полный шаг (предсказание + коррекция)
// func (w *EKFWrapper) Run(acc *models.ACCData, gyro *models.GYROData, gnss *models.GNSSData) (*models.EstimatedState, error) {
func (w *EKFWrapper) Run(u mat.Vector, z mat.Vector) (*models.EstimatedState, error) {

	// Получаем текущее состояние
	x := w.lastEstimate.Val()

	// Выполняем полный шаг
	estimate, err := w.ekf.Run(x, u, z)
	if err != nil {
		return nil, fmt.Errorf("ошибка выполнения шага EKF: %v", err)
	}

	w.lastEstimate = estimate

	return w.estimateToState(estimate), nil
}

// estimateToState преобразует оценку в EstimatedState
func (w *EKFWrapper) estimateToState(est filter.Estimate) *models.EstimatedState {
	state := &models.EstimatedState{
		Timestamp: time.Now(), // В реальном приложении использовать временную метку данных
	}

	val := est.Val()
	cov := est.Cov()

	// Извлекаем состояние
	state.PositionX = val.At(0, 0)
	state.PositionY = val.At(1, 0)
	state.PositionZ = val.At(2, 0)

	state.QuaternionW = val.At(6, 0)
	state.QuaternionX = val.At(7, 0)
	state.QuaternionY = val.At(8, 0)
	state.QuaternionZ = val.At(9, 0)

	// Извлекаем ковариации
	state.CovarianceXX = cov.At(0, 0)
	state.CovarianceYY = cov.At(1, 1)
	state.CovarianceZZ = cov.At(2, 2)

	state.CovarianceQwQw = cov.At(6, 6)
	state.CovarianceQxQx = cov.At(7, 7)
	state.CovarianceQyQy = cov.At(8, 8)
	state.CovarianceQzQz = cov.At(9, 9)

	return state
}

// GetState возвращает текущее состояние
func (w *EKFWrapper) GetState() *models.EstimatedState {
	return w.estimateToState(w.lastEstimate)
}
