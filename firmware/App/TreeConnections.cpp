/*
* TreeConnections.cpp
*
*  Created on: May 12, 2025
*      Author: Varalakshmi
*/

#include "TreeConnections.h"

#ifdef Master
    #include "I2CInstance.h"
#endif

#ifdef Master
    I2CInstance* getI2CBusForInterface(const char* interfaceName) {
        if (strncmp(interfaceName, "I2C1", 4) == 0) {
            return &i2c1;
        }
        else if (strncmp(interfaceName, "I2C2", 4) == 0) {
            return &i2c2;
        } else {
            return nullptr;
        }
    }
#endif

// Constructor to set motor pins
TreeConnections::TreeConnections() {
    // TODO Auto-generated constructor stub
}

const Connection* getConnections(byte id, int& size) {
    switch (id) {
        case Stainer_Master_PCB_ID:
            size = sizeof(StainerMaster) / sizeof(StainerMaster[0]);
            return StainerMaster;
        case Stainer_Gantry_PCB_ID:
            size = sizeof(StainerGantry) / sizeof(StainerGantry[0]);
            return StainerGantry;
        case Nozzle_Mount_PCB_ID:
            size = sizeof(NozzleMount) / sizeof(NozzleMount[0]);
            return NozzleMount;
        default:
            size = 0;
            return nullptr;
    }
}

const char* findInterfaceDFS(byte currentID, byte targetID, std::set<byte>& visited) {
    if (visited.count(currentID)) return nullptr;
    visited.insert(currentID);

    int size = 0;
    const Connection* conns = getConnections(currentID, size);
    if (!conns) return nullptr;

    for (int i = 0; i < size; ++i) {
        if (conns[i].deviceID == targetID) {
            return conns[i].interface; // Direct connection
        }
    }

    for (int i = 0; i < size; ++i) {
        byte nextID = conns[i].deviceID;
        const char* iface = findInterfaceDFS(nextID, targetID, visited);
        if (iface) {
            return conns[i].interface; // First hop toward the path
        }
    }

    return nullptr; // Not found
}

const char* getInterfaceForDevice(byte currentMasterID, byte targetID) {
    std::set<byte> visited;
    return findInterfaceDFS(currentMasterID, targetID, visited);
}

byte findConnectedID(byte currentPCBID, const char* interfaceName) {
    if (!interfaceName) return 0;

    int size = 0;
    const Connection* conns = getConnections(currentPCBID, size);
    if (!conns || size <= 0) return 0;

    for (int i = 0; i < size; ++i) {
        if (conns[i].interface && strcmp(conns[i].interface, interfaceName) == 0) {
            return conns[i].deviceID;
        }
    }

    return 0; // not found
}

const char* getMotorType(byte currentPCBID, byte motorID) {
    int size;
    const Connection* connections = getConnections(currentPCBID, size);
    if (!connections) return nullptr;

    for (int i = 0; i < size; ++i) {
        if (connections[i].deviceID == motorID && strncmp(connections[i].interface, "Motor", 5) == 0) {
            return connections[i].interface;
        }
    }
    return nullptr; // Not found
}

TreeConnections::~TreeConnections() {
    // TODO Auto-generated destructor stub
}