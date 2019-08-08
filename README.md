# Motorcycle_street_parking_sensor
## Project still in-Progress

## Background
Motorcycles are very small and hard to see, this is especially problematic in parking situations. In an automobile you normally don't need to worry about someone hitting your car while it is parked, but this is a very real scenario for motorcycles. In urban areas/apartments where there is only street parking avaliable it is easy for drivers not to see a parked motorcycle while parallel parking. This mistake ultimately leads to knocking a motorcycle over. This simple mistake can result in costly motorcycle repairs even at very low speeds. Many motorcyclists have likely come out to find their motorcycle damaged on the ground, or know a friend who has. This project is meant to create an alert system for other drivers when they are getting to close to a motorcycle while parking. It uses flashing lights and a siren to alert the driver of the motorcycles presence, and hopefully prevent collision.

## Technical Details
*Processor: STM32G0
  * Running RTOS
  * at Least 1 DAC
  * at least 3 timers
*sensors: 
  *2x Sonar
  *ambient light sensor
*Alert system
  *LED strobe light(at least one per side L/R)
  *speaker (for audio alerts)
  
