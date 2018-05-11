#include <nRF5x_BLE_API.h>

BLE           ble;


Ticker                    ticker_task1;
int done = 0;
static uint8_t characteristic_is_fond = 0;
static uint8_t descriptor_is_found = 0;
static DiscoveredCharacteristic            chars_hrm;
static DiscoveredCharacteristicDescriptor  desc_of_chars_hrm(NULL,GattAttribute::INVALID_HANDLE,GattAttribute::INVALID_HANDLE,UUID::ShortUUIDBytes_t(0));

static void scanCallBack(const Gap::AdvertisementCallbackParams_t *params);
static void discoveredServiceCallBack(const DiscoveredService *service);
static void discoveredCharacteristicCallBack(const DiscoveredCharacteristic *chars);
static void discoveryTerminationCallBack(Gap::Handle_t connectionHandle);
static void discoveredCharsDescriptorCallBack(const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *params);
static void discoveredDescTerminationCallBack(const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *params) ;

uint32_t ble_advdata_parser(uint8_t type, uint8_t advdata_len, uint8_t *p_advdata, uint8_t *len, uint8_t *p_field_data) {
  uint8_t index=0;
  uint8_t field_length, field_type;

  while(index<advdata_len) {
    field_length = p_advdata[index];
    field_type   = p_advdata[index+1];
    if(field_type == type) {
      memcpy(p_field_data, &p_advdata[index+2], (field_length-1));
      *len = field_length - 1;
      return NRF_SUCCESS;
    }
    index += field_length + 1;
  }
  return NRF_ERROR_NOT_FOUND;
}

void startDiscovery(uint16_t handle) {
  ble.gattClient().launchServiceDiscovery(handle, discoveredServiceCallBack, discoveredCharacteristicCallBack);
}

static void scanCallBack(const Gap::AdvertisementCallbackParams_t *params) {
  uint8_t index;

  Serial.println("Scan CallBack ");
  Serial.print("PerrAddress: ");
  for(index=0; index<6; index++) {
    Serial.print(params->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  Serial.print("The Rssi : ");
  Serial.println(params->rssi, DEC);

  Serial.print("The adv_data : ");
  Serial.println((const char*)params->advertisingData);

  uint8_t len;
  uint8_t adv_name[31];
  
  if( NRF_SUCCESS == ble_advdata_parser(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, params->advertisingDataLen, (uint8_t *)params->advertisingData, &len, adv_name) ) {
    Serial.print("Complete name len : ");
    Serial.println(len, DEC);
    Serial.print("Complete name is : ");
    Serial.println((const char*)adv_name);

    if(memcmp("LED", adv_name, 3) == 0x00) {
      Serial.println("Got device, stop scan ");
      ble.stopScan();
      ble.connect(params->peerAddr, BLEProtocol::AddressType::RANDOM_STATIC, NULL, NULL);
    }
  }
  Serial.println(" ");
}


void connectionCallBack( const Gap::ConnectionCallbackParams_t *params ) {
  uint8_t index;

  Serial.print("The conn handle : ");
  Serial.println(params->handle, HEX);

  Serial.print("  The peerAddr : ");
  for(index=0; index<6; index++) {
    Serial.print(params->peerAddr[index], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  startDiscovery(params->handle);
}

void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected, start to scanning");
  characteristic_is_fond = 0;
  descriptor_is_found = 0;
  ble.startScan(scanCallBack);
}

static void discoveredServiceCallBack(const DiscoveredService *service) {
  Serial.println("\r\n----Servuce Discovered");
  Serial.print("Service UUID             : ");
  if(service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
    Serial.println(service->getUUID().getShortUUID(), HEX);
  }
  else {
    uint8_t index;
    const uint8_t *uuid = service->getUUID().getBaseUUID();
    for(index=0; index<16; index++) {
      Serial.print(uuid[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  Serial.print("The service start handle : ");
  Serial.println(service->getStartHandle(), HEX);
  Serial.print("The service end handle   : ");
  Serial.println(service->getEndHandle(), HEX);
}

static void discoveredCharacteristicCallBack(const DiscoveredCharacteristic *chars) {
  Serial.println("\r\n----Characteristic Discovered");
  Serial.print("Chars UUID             : ");
  if(chars->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
    Serial.println(chars->getUUID().getShortUUID(), HEX);
    if(chars->getUUID().getShortUUID() == 0xC1){
      characteristic_is_fond = 1;
      chars_hrm = *chars;
    }
  }
  
  else {
    uint8_t index;
    const uint8_t *uuid = chars->getUUID().getBaseUUID();
    for(index=0; index<16; index++) {
      Serial.print(uuid[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }

  Serial.print("properties_read        : ");
  Serial.println(chars->getProperties().read(), DEC);
  Serial.print("properties_writeWoResp : ");
  Serial.println(chars->getProperties().writeWoResp(), DEC);
  Serial.print("properties_write       : ");
  Serial.println(chars->getProperties().write(), DEC);
  Serial.print("properties_notify      : ");
  Serial.println(chars->getProperties().notify(), DEC);

  Serial.print("declHandle             : ");
  Serial.println(chars->getDeclHandle(), HEX);
  Serial.print("valueHandle            : ");
  Serial.println(chars->getValueHandle(), HEX);
  Serial.print("lastHandle             : ");
  Serial.println(chars->getLastHandle(), HEX);

}

static void discoveryTerminationCallBack(Gap::Handle_t connectionHandle) {
  Serial.println("\r\n----discoveryTermination");
  Serial.print("connectionHandle       : ");
  Serial.println(connectionHandle, HEX);
  if(characteristic_is_fond == 1) {
    ble.gattClient().discoverCharacteristicDescriptors(chars_hrm, discoveredCharsDescriptorCallBack, discoveredDescTerminationCallBack);
  }
}

static void discoveredCharsDescriptorCallBack(const CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t *params) {

  if(params->descriptor.getUUID().getShortUUID() == 0x2901) {
    Serial.println("\r\n----discovered descriptor");
    Serial.print("Desriptor UUID         : ");
    Serial.println(params->descriptor.getUUID().getShortUUID(), HEX);
    descriptor_is_found = 1;
    desc_of_chars_hrm = params->descriptor;
    Serial.print("connectionHandle       : ");
    Serial.println(params->descriptor.getConnectionHandle(), HEX);
    Serial.print("descriptor Handle      : ");
    Serial.println(params->descriptor.getAttributeHandle(), HEX);
  }
}

static void discoveredDescTerminationCallBack(const CharacteristicDescriptorDiscovery::TerminationCallbackParams_t *params) {
  if(descriptor_is_found) {
    Serial.println("\r\n----discovery descriptor Termination");
    Serial.println("Open LED notify");
    uint16_t value = 0x0001;
    ble.gattClient().write(GattClient::GATT_OP_WRITE_REQ, chars_hrm.getConnectionHandle(), desc_of_chars_hrm.getAttributeHandle(), 2, (uint8_t *)&value);
    ble.gattClient().write(GattClient::GATT_OP_WRITE_REQ, chars_hrm.getConnectionHandle(), chars_hrm.getValueHandle(), 2, (uint8_t *)&value);
    done = 1;
  }

}

void onDataWriteCallBack(const GattWriteCallbackParams *params) {
  Serial.println("GattClient write call back ");
  
  Serial.print("connectionHandle       : ");
  Serial.println(chars_hrm.getConnectionHandle(), HEX);
  Serial.print("descriptor Handle      : ");
  Serial.println(desc_of_chars_hrm.getAttributeHandle(), HEX);
//  ble.gattClient().read(chars_hrm.getConnectionHandle(), chars_hrm.getValueHandle(), 0);
}


void onDataReadCallBack(const GattReadCallbackParams *params) {
  Serial.println("GattClient read call back ");
  Serial.print("The handle : ");
  Serial.println(params->handle, HEX);
  Serial.print("The offset : ");
  Serial.println(params->offset, DEC);
  Serial.print("The len : ");
  Serial.println(params->len, DEC);
  Serial.print("The data : ");
  for(uint8_t index=0; index<params->len; index++) {
    Serial.print( params->data[index], HEX);
  }
  Serial.println("");
}


void hvxCallBack(const GattHVXCallbackParams *params) {
  Serial.println("GattClient notify call back ");
  Serial.print("The len : ");
  Serial.println(params->len, DEC);
  for(unsigned char index=0; index<params->len; index++) {
    Serial.print(params->data[index], HEX);
  }
  Serial.println("");
}

void periodicCallback() {
  if (done){
    uint8_t value = 'a';
    ble.gattClient().write(GattClient::GATT_OP_WRITE_REQ, chars_hrm.getConnectionHandle(), chars_hrm.getValueHandle(), 1, (uint8_t *)&value);
   }
}

void setup() {
  Serial.begin(9600);
  Serial.println("BLE Central Demo ");

  ble.init();
  ble.onConnection(connectionCallBack);
  ble.onDisconnection(disconnectionCallBack);
  ble.gattClient().onServiceDiscoveryTermination(discoveryTerminationCallBack);
  ble.gattClient().onHVX(hvxCallBack);
  ble.gattClient().onDataWrite(onDataWriteCallBack);
  ble.gattClient().onDataRead(onDataReadCallBack);
  ble.setScanParams(1000, 200, 0, true);
  ble.startScan(scanCallBack);

  ticker_task1.attach(periodicCallback, 1);
}

void loop() {
  ble.waitForEvent();
}

