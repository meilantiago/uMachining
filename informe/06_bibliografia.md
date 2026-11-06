# 6. Bibliografía

Referencias candidatas, agrupadas por tema, con la parte del informe donde se citan. **Verificar edición, páginas y versión de cada hoja de datos antes de usarlas**: las hojas de datos cambian de revisión y no se tiene en la carpeta del proyecto la versión que se consultó.

## Hojas de datos y hardware

1. MagnTek. *MT6835: 21-bit high-resolution magnetic angle sensor*, hoja de datos. (Principio AMR, 21 bits, campo mínimo de 30 mT en la p. 9 según el cuaderno, bits de estado, ancho de banda.) → Introducción 1.3, desarrollo 2.2.
2. Toshiba. *TB6612FNG: driver IC for dual DC motor*, hoja de datos. (15 V máx., 1,2 A continuos por canal, modos de decaimiento.) → Desarrollo 2.2.
3. Raspberry Pi Ltd. *RP2040 Datasheet* y *Raspberry Pi Pico-series C/C++ SDK*. (Dos núcleos, spinlocks de hardware, `multicore_lockout`, sin FPU.) → Desarrollo 2.4.

## Proyecto de origen

4. 0x23. *MicroManipulatorStepper* (firmware `MotionControllerRP`), repositorio GitHub, licencia MIT. https://github.com/0x23/MicroManipulatorStepper → Introducción 1.1, desarrollo 2.4.
5. M. Ortner, L. G. Coliado Bandeira. "Magpylib: A free Python package for magnetic field computation". *SoftwareX* **11**, 100466 (2020). → Desarrollo 2.3.
6. AKM (Asahi Kasei Microdevices). *Rotation angle sensor tutorial: off-axis configuration*. https://www.akm.com/us/en/products/rotation-angle-sensor/tutorial/off-axis-configuration/ (citado en el cuaderno, 14/8). → Introducción 1.3.

## Física del sensor y del motor

7. T. R. McGuire, R. I. Potter. "Anisotropic magnetoresistance in ferromagnetic 3d alloys". *IEEE Transactions on Magnetics* **11**(4), 1018–1038 (1975). → Introducción 1.3 (AMR).
8. P. Ripka (ed.). *Magnetic Sensors and Magnetometers*. Artech House (2001). → Introducción 1.3 (alternativa general a la anterior).
9. P. Acarnley. *Stepping Motors: A Guide to Theory and Practice*, 4.ª ed. IET (2002). → Introducción 1.4 (motor híbrido, torque contra ángulo de carga).
10. R. Krishnan. *Permanent Magnet Synchronous and Brushless DC Motor Drives*. CRC Press (2010). → Introducción 1.4 (control por campo).

## Control

11. K. J. Åström, T. Hägglund. *Advanced PID Control*. ISA (2006). → Introducción 1.5 (PI en cascada, antisaturación, realimentación anticipada).
12. B. Armstrong-Hélouvry, P. Dupont, C. Canudas de Wit. "A survey of models, analysis tools and compensation methods for the control of machines with friction". *Automatica* **30**(7), 1083–1138 (1994). → Introducción 1.5, resultados 3.4 (ciclo límite por fricción estática, *hunting*, *stick-slip*).
13. D. A. Bristow, M. Tharayil, A. G. Alleyne. "A survey of iterative learning control". *IEEE Control Systems Magazine* **26**(3), 96–114 (2006). → Desarrollo 2.5, resultados 3.3 (aprendizaje del anti-cogging).

## Análisis de datos y métodos numéricos

14. S. Braun. "The extraction of periodic waveforms by time domain averaging". *Acustica* **32**, 69–77 (1975). → Desarrollo 2.7 (promedio sincrónico, plegado).
15. W. H. Press, S. A. Teukolsky, W. T. Vetterling, B. P. Flannery. *Numerical Recipes: The Art of Scientific Computing*, 3.ª ed. Cambridge University Press (2007), §2.4 (sistemas tridiagonales). → Desarrollo 2.5 (algoritmo de Thomas).

## Metrología

16. JCGM 200:2012. *International vocabulary of metrology — Basic and general concepts and associated terms (VIM)*, 3.ª ed. → Introducción 1.7.
17. JCGM 100:2008. *Evaluation of measurement data — Guide to the expression of uncertainty in measurement (GUM)*. → Conclusiones, trabajo futuro (presupuesto de incertidumbre).
18. ISO 230-2:2014. *Test code for machine tools — Part 2: Determination of accuracy and repeatability of positioning of numerically controlled axes*. → Conclusiones, trabajo futuro (ensayos E1–E4).
19. A. H. Slocum. *Precision Machine Design*. Prentice Hall (1992). → Introducción y trabajo futuro (errores de Abbe y de coseno, transmisión por husillo).

## Aplicación: micromaquinado láser ultrarrápido

20. R. R. Gattass, E. Mazur. "Femtosecond laser micromachining in transparent materials". *Nature Photonics* **2**, 219–225 (2008). → Introducción 1.1.
21. K. Sugioka, Y. Cheng. "Ultrafast lasers—reliable tools for advanced materials processing". *Light: Science & Applications* **3**, e149 (2014). → Introducción 1.1.

## Documentación propia (si se cita el repositorio)

22. F. Maresca, T. Meilán. *μMachining: firmware de control en lazo cerrado para un eje lineal de micromaquinado*. https://github.com/meilantiago/uMachining (versión del 06/11/2026). → Desarrollo 2.4, apéndice.
