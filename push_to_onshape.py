#!/usr/bin/env python3
"""
Upload STEP models to Onshape workspace and auto-translate into Part Studios.
"""

import os
import sys
import json
import time
from onshape_client.client import Client
from onshape_client.oas.models.bt_document_params import BTDocumentParams

def get_onshape_client():
    creds_path = os.path.expanduser("~/.onshape/credentials.json")
    with open(creds_path) as f:
        creds = json.load(f)
    return Client(configuration={
        "base_url": creds.get("baseUrl", "https://cad.onshape.com"),
        "access_key": creds["accessKey"],
        "secret_key": creds["secretKey"]
    })

def upload_and_translate(client, did, wid, file_path):
    filename = os.path.basename(file_path)
    print(f"Uploading {filename}...")
    with open(file_path, "rb") as f:
        res = client.blob_elements_api.upload_file_create_element(
            did=did,
            wid=wid,
            file=f,
            translate=True,
            format_name="STEP",
            unit="millimeter",
            _preload_content=False
        )
    data = json.loads(res.data.decode("utf-8"))
    tid = data.get("translationId")
    if not tid:
        print(f"Upload completed without translation: {data.get('id')}")
        return data.get("id")

    print(f"Waiting for Onshape CAD translation (tid: {tid})...")
    for _ in range(30):
        t_res = client.translation_api.get_translation(tid=tid, _preload_content=False)
        t_data = json.loads(t_res.data.decode("utf-8"))
        state = t_data.get("requestState")
        if state == "DONE":
            created = t_data.get("resultElementIds", [])
            print(f"Translation DONE -> Part Studio Element: {created}")
            return created
        elif state == "FAILED":
            print(f"Translation failed: {t_data.get('failureReason')}")
            return None
        time.sleep(1)
    print("Translation timed out.")
    return None

def main():
    doc_name = sys.argv[1] if len(sys.argv) > 1 else "DC180_Motor_Mount_Clamp"
    client = get_onshape_client()

    print(f"Creating Onshape document: '{doc_name}'...")
    params = BTDocumentParams(name=doc_name, is_public=True)
    doc = client.documents_api.create_document(bt_document_params=params)
    did = doc.id
    wid = doc.default_workspace.id
    print(f"Document created! did={did}, wid={wid}")

    step_files = [
        "dc180_mount_clamp_assembly.step",
        "dc180_mount_clamp_base.step",
        "dc180_mount_clamp_cap.step"
    ]

    for fname in step_files:
        fpath = os.path.join(os.path.dirname(__file__), fname)
        if os.path.exists(fpath):
            upload_and_translate(client, did, wid, fpath)

    # Delete default empty Part Studio 1 if exists
    res = client.documents_api.get_elements_in_document(did=did, wvm="w", wvmid=wid, _preload_content=False)
    for e in json.loads(res.data.decode("utf-8")):
        if e["name"] == "Part Studio 1" and e["elementType"] == "PARTSTUDIO":
            try:
                client.elements_api.delete_element(did=did, wid=wid, eid=e["id"])
                print("Cleaned up default empty Part Studio 1.")
            except Exception:
                pass

    doc_url = f"https://cad.onshape.com/documents/{did}/w/{wid}"
    print(f"\nSUCCESS! Onshape Document URL:\n{doc_url}")

if __name__ == "__main__":
    main()
