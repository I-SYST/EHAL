/**-------------------------------------------------------------------------
@file	ble_srvc_discovery.pp

@brief	BLE central helper : peripheral discovery


@author	Hoang Nguyen Hoan
@date	Jan. 15, 2019
@license

Copyright (c) 2019, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "ble.h"
#include "nrf_section_iter.h"
#include "nrf_sdh_ble.h"
#include "ble_srv_common.h"

#include "ble_app.h"
#include "ble_dev.h"

void BlePeriphDiscEvtHandler(ble_evt_t const *p_ble_evt, void *p_context);

static BLEPERIPH_DEV *s_pBlePeriphData = NULL;

__attribute__ ((section("." "sdh_ble_observers1"))) __attribute__((used))
static nrf_sdh_ble_evt_observer_t s_BlePeriphDiscObs = {
	.handler   = BlePeriphDiscEvtHandler,
	.p_context = (void*)&s_pBlePeriphData
};

static int s_CurSrvcHdl = 1;
static ble_uuid_t *s_CurSrvcUuid = NULL;
static ble_gatt_db_srv_t s_CurSrvcDisc;
static int s_CurSrvcIdx = 0;
static int s_CurCharIdx = 0;
static ble_gattc_handle_range_t s_CurRange;

__WEAK void BleDevDiscovered(BLEPERIPH_DEV *pDev)
{

}

bool BleAppDiscoverDevice(BLEPERIPH_DEV * const pDev)
{
	s_pBlePeriphData = pDev;
	s_CurSrvcIdx = 0;
	s_CurCharIdx = 0;

    uint32_t err_code = sd_ble_gattc_primary_services_discover(pDev->ConnHdl, 1, NULL);

    return err_code == NRF_SUCCESS;
}

int BleDevFindService(BLEPERIPH_DEV * const pDev, uint16_t Uuid)
{
    for (int i = 0; i < pDev->NbSrvc; i++)
    {
    	if (pDev->Services[i].srv_uuid.uuid == Uuid)
    	{
    		return i;
    	}
    }

    return -1;
}

int BleDevFindCharacteristic(BLEPERIPH_DEV * const pDev, int SrvcIdx, uint16_t Uuid)
{
    for (int i = 0; i < pDev->Services[SrvcIdx].char_count; i++)
    {
    	if (pDev->Services[SrvcIdx].charateristics[i].characteristic.uuid.uuid == Uuid)
    	{
    		return i;
    	}
    }

    return -1;
}

/*
bool BleDevDiscoverService(uint16_t ConnHandle, ble_uuid_t * const pSrvcUuid)
{
	//memset()
	return true;
}

bool BleDevDiscoverCharacteristics(uint16_t ConnHandle, ble_gattc_handle_range_t const *p_handle_range)
{
	uint32_t err = sd_ble_gattc_characteristics_discover(ConnHandle, p_handle_range);

	printf("sd_ble_gattc_characteristics_discover %x %x %x\r\n", err, ConnHandle, *p_handle_range);
	return err == NRF_SUCCESS;
}

bool BlePeriphDiscInit()
{
	s_CurSrvcHdl = 1;

	return true;
}

void BlePeriphDiscService(uint16_t ConnHandle, ble_uuid_t * const pSrvcUuid)
{
	if (pSrvcUuid != NULL)
	{
		printf("New uuid %x\r\n", pSrvcUuid);
		s_CurSrvcUuid = pSrvcUuid;
	}

    uint32_t err_code = sd_ble_gattc_primary_services_discover(ConnHandle, s_CurSrvcHdl, s_CurSrvcUuid);
    printf("BlePeriphDiscService err = %x %x\r\n", err_code, s_CurSrvcUuid);

}
*/

void BlePeriphDiscEvtHandler(ble_evt_t const *p_ble_evt, void *p_context)
{
	BLEPERIPH_DEV *periph = *(BLEPERIPH_DEV**)p_context;
	ble_gattc_evt_t    const * p_ble_gattc_evt = &(p_ble_evt->evt.gattc_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
        	{
        		if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
                {
        			ble_gattc_evt_prim_srvc_disc_rsp_t const * p_prim_srvc_disc_rsp_evt =
                		&(p_ble_gattc_evt->params.prim_srvc_disc_rsp);

					for (int i = 0; i < p_prim_srvc_disc_rsp_evt->count; i++, periph->NbSrvc++)
					{
	        			periph->Services[periph->NbSrvc].char_count = 0;
	        			periph->Services[periph->NbSrvc].srv_uuid = p_prim_srvc_disc_rsp_evt->services[i].uuid;
	        			periph->Services[periph->NbSrvc].handle_range = p_prim_srvc_disc_rsp_evt->services[i].handle_range;

						s_CurSrvcHdl = p_prim_srvc_disc_rsp_evt->services[i].handle_range.end_handle;
					}

					s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
					s_CurRange.start_handle++;	// the service uses 1 handle already
					uint32_t err = sd_ble_gattc_characteristics_discover(periph->ConnHdl, &s_CurRange);
                }
        		else
        		{
        			BleDevDiscovered(periph);
        		}
        	}
        	break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
            {
                ble_gattc_evt_char_disc_rsp_t const * p_char_disc_rsp_evt;

                p_char_disc_rsp_evt = &(p_ble_gattc_evt->params.char_disc_rsp);

                for (int i = 0; i < p_char_disc_rsp_evt->count; i++, periph->Services[s_CurSrvcIdx].char_count++)
                {
                    memcpy(&periph->Services[s_CurSrvcIdx].charateristics[periph->Services[s_CurSrvcIdx].char_count].characteristic,
                    	   &p_char_disc_rsp_evt->chars[i], sizeof(ble_gattc_char_t));
                    periph->Services[s_CurSrvcIdx].charateristics[periph->Services[s_CurSrvcIdx].char_count].cccd_handle = BLE_GATT_HANDLE_INVALID;
                    periph->Services[s_CurSrvcIdx].charateristics[periph->Services[s_CurSrvcIdx].char_count].ext_prop_handle = BLE_GATT_HANDLE_INVALID;
                    periph->Services[s_CurSrvcIdx].charateristics[periph->Services[s_CurSrvcIdx].char_count].user_desc_handle = BLE_GATT_HANDLE_INVALID;
                    periph->Services[s_CurSrvcIdx].charateristics[periph->Services[s_CurSrvcIdx].char_count].report_ref_handle = BLE_GATT_HANDLE_INVALID;
                    s_CurRange.start_handle = p_char_disc_rsp_evt->chars[i].handle_value + 1;
                }

				uint32_t err = sd_ble_gattc_characteristics_discover(periph->ConnHdl, &s_CurRange);
				//printf("char err = %x\r\n", err);
            }
            else
            {
            	//printf("Char Status = %x, range %x\r\n", p_ble_gattc_evt->gatt_status, s_CurRange);
    			if (s_CurRange.start_handle <= s_CurRange.end_handle)
    			{
    				s_CurCharIdx = 0;
    				s_CurRange.start_handle = periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].characteristic.handle_value + 1;
        			uint32_t err = sd_ble_gattc_descriptors_discover(periph->ConnHdl, &s_CurRange);
    			}
    			else
    			{
    				s_CurSrvcIdx++;
					if (s_CurSrvcIdx < periph->NbSrvc)
					{
						s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
    					s_CurRange.start_handle++;	// the service uses 1 handle already
						uint32_t err = sd_ble_gattc_characteristics_discover(periph->ConnHdl, &s_CurRange);
					}
					else
					{
						uint32_t err_code = sd_ble_gattc_primary_services_discover(periph->ConnHdl, s_CurRange.end_handle, NULL);
					}
    			}
            }
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
            {
            	const ble_gattc_evt_desc_disc_rsp_t * p_desc_disc_rsp_evt = &(p_ble_gattc_evt->params.desc_disc_rsp);

            	for (int i = 0; i < p_desc_disc_rsp_evt->count; i++)
            	{
                    switch (p_desc_disc_rsp_evt->descs[i].uuid.uuid)
                    {
                        case BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
                            periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].cccd_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP:
                            periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].ext_prop_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_DESCRIPTOR_CHAR_USER_DESC:
                            periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].user_desc_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;

                        case BLE_UUID_REPORT_REF_DESCR:
                            periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].report_ref_handle = p_desc_disc_rsp_evt->descs[i].handle;
                            s_CurRange.start_handle++;
                            break;
                        case BLE_UUID_CHARACTERISTIC:
                        	{
                        		// It's a characteristic
                    			s_CurCharIdx++;
            					if (s_CurCharIdx < periph->Services[s_CurSrvcIdx].char_count)
            					{
            	    				s_CurRange.start_handle = periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].characteristic.handle_value + 1;
                        			uint32_t err = sd_ble_gattc_descriptors_discover(periph->ConnHdl, &s_CurRange);
                        			return;
                        		//	break;
                    			}
//                        		uint32_t err = sd_ble_gattc_characteristics_discover(periph->ConnHdl, &s_CurRange);
                        	}
                        	break;
                        default:
                        	;
                    }
            	}
        		if (s_CurRange.start_handle < s_CurRange.end_handle)
            	{
        			s_CurCharIdx++;
					if (s_CurCharIdx < periph->Services[s_CurSrvcIdx].char_count)
					{
	    				s_CurRange.start_handle = periph->Services[s_CurSrvcIdx].charateristics[s_CurCharIdx].characteristic.handle_value + 1;
            			uint32_t err = sd_ble_gattc_descriptors_discover(periph->ConnHdl, &s_CurRange);
            			break;
        			}
            	}
				else
				{
					s_CurSrvcIdx++;
					if (s_CurSrvcIdx < periph->NbSrvc)
					{
						s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
						uint32_t err = sd_ble_gattc_characteristics_discover(periph->ConnHdl, &s_CurRange);
					}
					else
					{
						uint32_t err_code = sd_ble_gattc_primary_services_discover(periph->ConnHdl, s_CurRange.end_handle, NULL);

					}
				}



            }
            else
            {
    			s_CurCharIdx++;
				if (s_CurCharIdx < periph->Services[s_CurSrvcIdx].char_count)
				{
        			uint32_t err = sd_ble_gattc_descriptors_discover(periph->ConnHdl, &s_CurRange);
        			break;
    			}
				else
				{
					s_CurSrvcIdx++;
					if (s_CurSrvcIdx < periph->NbSrvc)
					{
						s_CurRange = periph->Services[s_CurSrvcIdx].handle_range;
						uint32_t err = sd_ble_gattc_characteristics_discover(periph->ConnHdl, &s_CurRange);
					}
					else
					{
						uint32_t err_code = sd_ble_gattc_primary_services_discover(periph->ConnHdl, s_CurRange.end_handle, NULL);
					}
				}
            }
            break;

        default:
            break;
    }

}
