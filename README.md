# Introduction

![image](https://user-images.githubusercontent.com/48565455/179184357-ba784f1b-9d97-44d3-a038-2ed4c0fcc63f.png)

The purpose of this project was to create a business use case for an Internet of Things (IoT) device. 
Internet of Pets introduced an adjustable pet-friendly collar that is lightweight and recognizable to the eye. 
Within that collar, there is a GPS tracker to track a pet’s whereabouts to bring end-users at ease. 
Internet of Pets utilized two different types of IoT prototypes based on two different connectivities in which the main distinction was the type of board used. The Arduino MKR FOX 1200 was utilized for Sigfox connectivity while the Arduino MKR 1010 Wi-Fi was used for Wi-Fi connectivity.
Within SigFox, Internet of Pets is currently utilizing Sigfox callbacks with the bidirectional subtype to send data to the thinger.io and Microsoft Azure platform.
Within the WiFi module, we utilized the Arduino Cloud. 
Unfortunately, the Wi-Fi module led to a shallow proof of concept due to tracking feasibility. It is limited in range since a customer would need to have a constant Wi-Fi signal for the collar to track their pet.

# Dashboard (Azure and PowerBI)
The standard procedure for Internet of Pets of setting up the connection from Sigfox to Azure was creating a specified domain account for this startup and accessing four different resources to then have the data be visualized via the premium dashboard. Firstly, they had to connect, monitor, and configure the IoT device through the Azure IoT Hub which manages billions of IoT devices.
Secondly, as soon as they connected and captured the data it would run through Azure Stream Analytics to run real-time analytics on their data.
Thirdly, the data was collected and stored in the Microsoft Azure SQL Database. Finally, the coordinates were shown via the dashboard from Power BI. 
We implemented a number features via Azure to suit a better perspective for the customer's pet to better understand the pet's health.

![image](https://user-images.githubusercontent.com/48565455/179183220-4e1adef2-ef99-49a2-96f1-52958b8976a6.png)

