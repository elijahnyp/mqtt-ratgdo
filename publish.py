import os
import requests
import re

SECRET = os.getenv('CLIENT_SECRET')
REFRESH = os.getenv('REFRESH_TOKEN')

resp = requests.post(
    url='https://cloak.elijahmm.com/auth/realms/Wangwood/protocol/openid-connect/token',
    data={
        'client_id': 'firmware.wangwood.house',
        'grant_type': 'refresh_token',
        'refresh_token': REFRESH,
        'client_secret': SECRET
        }
    )

if(resp.status_code < 200 or resp.status_code > 299):
    print(f"ERROR AUTHENTICATING: CODE {resp.status_code}")
    exit(1)

TOKEN = resp.json().get('access_token')

fwmap = {}
def get_map():
    resp = requests.get(
    url='https://firmware.wangwood.house/admin/firmware',
    headers={'Authorization': f"Bearer {TOKEN}"}
    )

    if(resp.status_code != 200):
        print(f"ERROR GETTING EXISTING FIRMWARE VERSIONS - SOMETHING WENT WRONG")
        print(resp.content)
        exit(1)

    for f in resp.json():
        fwmap[f.get('device')] = f.get('id')

get_map()

def create_firmware(firmware):
    resp = requests.post(
        url='https://firmware.wangwood.house/admin/new_firmware',
        headers={'Authorization': f"Bearer {TOKEN}"},
        data={
            "name": firmware
        }
    )
    if(resp.status_code != 200):
        print(f"Unable to create new firmware model {firmware}")
        exit(1)

for (root, _, files) in os.walk('out'):
    for f in files:
        try:
            fp = f.split('.')
            if(fp[1].lower() != 'bin'):
                continue
            id = fwmap.get(fp[0])
            if(id is None):
                create_firmware(fp[0])
                get_map()
                id=fwmap.get(fp[0])
                if(id is None):
                    print("SOMETHING SERIOUSLY WRONG - CREATED DEVICE BUT NOT RETURNED")
                    exit(1)
            print(f"uploading {root}/{f}");
            data={
                    "version": "",
                    "id": fwmap[fp[0]]
                }
            files={
                'firmware': open(f'{root}/{f}', 'rb')
            }
            try:
                resp = requests.post(
                    url='https://firmware.wangwood.house/admin/new_version',
                    headers={'Authorization': f"Bearer {TOKEN}"},
                    data=data,
                    files=files
                )
            except Exception as e:
                print(str(e))
            try:
                if(resp.json()['status'].lower() != 'updated'):
                    print(f"failed to upload firmware: {resp.status_code}")
                    print(f"response: {resp.content}")
                    exit(1)
            except Exception as e:    
                if(resp.status_code != 200):
                    print(f"failed to upload firmware: {resp.status_code}")
                    print(f"response: {resp.content}")
                    exit(1)
                else:
                    print(f"invalid response to upload - aborting.  Likely credentials")
                    print(f"data: {data}")
                    print(f"files: {files}")
                    print(f"token: {TOKEN}")
                    print(str(e))
                    exit(1)
            print("success")
        except Exception as e:
            print(e)
            continue

    break



# curl -X POST 'https://firmware.wangwood.house/admin/new_version' \
# -H 'Content-Type: application/x-www-form-urlencoded' \
# -H "Authorization: Bearer ${token}" \
# -d 'id=7f177264-a893-11eb-8d69-ba4e65b51aa3' \
# -d 'version='0.00' \