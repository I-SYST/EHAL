//
//  ViewController.swift
//  SensorTagDemo
//
//  Created by Nguyen Hoan Hoang on 2017-10-22.
//  Copyright © 2017 I-SYST inc. All rights reserved.
//

import Cocoa
import AppKit//QuartzCore
import CoreBluetooth
import simd

class ViewController: NSViewController, CBCentralManagerDelegate {

    var bleCentral : CBCentralManager!

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        bleCentral = CBCentralManager(delegate: self, queue: DispatchQueue.main)
   	}

    override var representedObject: Any? {
        didSet {
        // Update the view, if already loaded.
        }
    }

    //@IBOutlet weak var graphView : GraphView!
    @IBOutlet weak var tempLabel: NSTextField!
    @IBOutlet weak var humiLabel : NSTextField!
    @IBOutlet weak var pressLabel : NSTextField!
    @IBOutlet weak var rssiLabel : NSTextField!
    
    // MARK: BLE Central
    
    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData : [String : Any],
                        rssi RSSI: NSNumber) {
        print("PERIPHERAL NAME: \(String(describing: peripheral.name))\n AdvertisementData: \(advertisementData)\n RSSI: \(RSSI)\n")
        
        //print("UUID DESCRIPTION: \(peripheral.identifier.uuidString)\n")
        
        //print("IDENTIFIER: \(peripheral.identifier)\n")
        
        if advertisementData[CBAdvertisementDataManufacturerDataKey] == nil {
            return
        }
        
        //sensorData.text = sensorData.text + "FOUND PERIPHERALS: \(peripheral) AdvertisementData: \(advertisementData) RSSI: \(RSSI)\n"
        var manId = UInt16(0)
        (advertisementData[CBAdvertisementDataManufacturerDataKey] as! NSData).getBytes(&manId, range: NSMakeRange(0, 2))
        if manId != 0x177 {
            return
        }
        
        var type = UInt8(0)
        (advertisementData[CBAdvertisementDataManufacturerDataKey] as! NSData).getBytes(&type, range: NSMakeRange(2, 1))
        if (type != 1) {
            return
        }
        
        var press = Int32(0)
        (advertisementData[CBAdvertisementDataManufacturerDataKey] as! NSData).getBytes(&press, range: NSMakeRange(3, 4))
        pressLabel.stringValue = String(format:"%.3f KPa", Float(press) / 1000.0)
        
        var temp = Int16(0)
        (advertisementData[CBAdvertisementDataManufacturerDataKey] as! NSData).getBytes(&temp, range: NSMakeRange(7, 2))
        tempLabel.stringValue = String(format:"%.2f C", Float(temp) / 100.0)
        
        var humi = UInt16(0)
        (advertisementData[CBAdvertisementDataManufacturerDataKey] as! NSData).getBytes(&humi, range: NSMakeRange(9, 2))
        humiLabel.stringValue = String(format:"%d%%", humi / 100)
        
        rssiLabel.stringValue = String( describing: RSSI)
        //graphView.add(double3(Double(temp) / 100.0, Double(press) / 100000.0, Double(humi) / 100.0))
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        
        peripheral.discoverServices(nil)
        print("Connected to peripheral")
        
        
    }
    
    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        print("disconnected from peripheral")
        
        
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
    }
    
    func scanPeripheral(_ sender: CBCentralManager)
    {
        print("Scan for peripherals")
        bleCentral.scanForPeripherals(withServices: nil, options: nil)
    }
    
    @objc func centralManagerDidUpdateState(_ central: CBCentralManager) {
        
        switch central.state {
            
        case .poweredOff:
            print("CoreBluetooth BLE hardware is powered off")
            //self.sensorData.text = "CoreBluetooth BLE hardware is powered off\n"
            break
        case .poweredOn:
            print("CoreBluetooth BLE hardware is powered on and ready")
            //self.sensorData.text = "CoreBluetooth BLE hardware is powered on and ready\n"
            // We can now call scanForBeacons
            //let lastPeripherals = central.retrieveConnectedPeripherals(withServices: nil)
            
            //if lastPeripherals.count > 0 {
                // let device = lastPeripherals.last as CBPeripheral;
                //connectingPeripheral = device;
                //centralManager.connectPeripheral(connectingPeripheral, options: nil)
            //}
            scanPeripheral(central)
            //bleCentralManager.scanForPeripherals(withServices: [NEB_SERVICE_UUID], options: nil)
            break
        case .resetting:
            print("CoreBluetooth BLE hardware is resetting")
            //self.sensorData.text = "CoreBluetooth BLE hardware is resetting\n"
            break
        case .unauthorized:
            print("CoreBluetooth BLE state is unauthorized")
            //self.sensorData.text = "CoreBluetooth BLE state is unauthorized\n"
            
            break
        case .unknown:
            print("CoreBluetooth BLE state is unknown")
            //self.sensorData.text = "CoreBluetooth BLE state is unknown\n"
            break
        case .unsupported:
            print("CoreBluetooth BLE hardware is unsupported on this platform")
            //self.sensorData.text = "CoreBluetooth BLE hardware is unsupported on this platform\n"
            break
            
        default:
            break
        }
    }

}

