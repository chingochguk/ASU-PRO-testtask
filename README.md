# ASU-PRO-testtask
Данный проект создан в программной среде IAR Embedded Workbench 6.30.
      Для базового конфигурирования микроконтроллера использовалось приложение STM32CubeMX.
      Документация создавалась в Doxygen. Микроконтроллер STM32F103C8T6.

 В проекте реализовано:
   1) Прием сообщений по UART.
   2) Обработка сообщений оканчивающихся символом NULL.
   3) Выполнение команд содержащихся в сообщениях и отправка ответа.
   4) Измерение напряжения с помощью АЦП с дискретностью 1000Гц.( Запуск
      начала измерения происходит в обработчике прерывания таймера, который и
      задает частоту дискретизации).
   5) Считывание с помощью DMA значения АЦП и передача его в регистр сравнения
      таймера, который выдает сигнал ШИМ. Скважность сигнала ШИМ зависит от
      значения напряжения АЦП.(На микроконтроллере STM32F103C8T6 нет ЦАП,
      поэтому используется этот вариант.)
   6) Подсчет среднего значения сигнала АЦП и вывод его в качестве ответа на команду.
   7) Использование ОСРВ FreeRTOS.
   8) Использование 3-ех потоков (задач) ОС.(ReceiveTask, LedTask, MovingAverageTask)
   9) Использование очередей, семафоров.

Для запуска проекта в программной среде IAR Embedded Workbench выбрать файл Project.eww из папки EWARM.
Для запуска проекта в STM32CubeMX выбрать first.ioc