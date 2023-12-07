Funding: This work was supported by the National Centre for Research and Development; Grant reference is POIR.01.01.01-00-0824/17 awarded to SensoriumLab sp. z o.o., Poland.

The repository contains the source code of the embedded software for the medical device PLEFIND, designed for analyzing the level of pleural effusion. 
PLEFIND was developed by SensoriumLab as part of the NCBR project - "System for non-invasive, outpatient diagnosis and monitoring of effusion into the pleural cavity."

PLEFIND device is equipped with STM32F446 microcontroller, which controls the array of emitters and detectors operating in the near-infrared spectrum. The system allows for device control, adjustment of emitted light power, and gain values in a dual detection channel. The Bluetooth Low Energy communication module was used to enable remote control of the device and wireless transmission of pre-processed signals from detectors to an external application for further analysis and presentation of measurement results. Signal processing involves noise reduction algorithms based on the orthogonality of sinusoidal functions (lock-in amplifier).






