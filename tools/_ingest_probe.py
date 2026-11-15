import socket,struct,time,json,urllib.request
HOST,PORT="185.33.234.10",1883
TOPIC=b"hexnet/v1/2/14/telemetry"
def enc(s):
 b=s if isinstance(s,bytes) else s.encode(); return struct.pack("!H",len(b))+b
s=socket.create_connection((HOST,PORT),8); s.settimeout(10)
rem=b"\x00\x04MQTT\x04\x02\x00\x3c"+enc(b"ingest-probe")
s.send(b"\x10"+bytes([len(rem)])+rem); s.recv(4)
pid=1; rem=struct.pack("!H",pid)+enc(TOPIC)+b"\x00"; s.send(b"\x82"+bytes([len(rem)])+rem); s.recv(5)
buf=b""; end=time.time()+10; payload=None
while time.time()<end and payload is None:
  try:
    chunk=s.recv(8192)
    if not chunk: break
    buf+=chunk
    while len(buf)>=2:
      if (buf[0]>>4)!=3:
        buf=buf[1:]; continue
      mul=1;rl=0;i=1
      while i<len(buf):
        e=buf[i];i+=1;rl+=(e&127)*mul;mul*=128
        if not (e&128): break
      else: break
      if len(buf)<i+rl: break
      pkt=buf[i:i+rl]; buf=buf[i+rl:]
      tlen=struct.unpack("!H",pkt[:2])[0]; payload=pkt[2+tlen:]
  except socket.timeout: break
s.close()
j=json.loads(payload)
print("got payload len", len(payload), "ts", j.get("ts"))
# EMQX-style body with object payload
body_obj={"topic":"hexnet/v1/2/14/telemetry","payload":j,"clientid":"hexnet-io-98a316dd8540","username":"","event":"message.publish"}
body=json.dumps(body_obj,separators=(",",":")).encode()
req=urllib.request.Request("https://myhexnet.com/api/mqtt/webhook-ingest.php", data=body, headers={"Content-Type":"application/json","X-Hexnet-Mqtt-Secret":"x-hexnet-2026-secret"}, method="POST")
with urllib.request.urlopen(req, timeout=20) as r:
  print("ingest", r.status, r.read().decode())
# also simulate-parse
req2=urllib.request.Request("https://myhexnet.com/api/mqtt/webhook-simulate-parse.php", data=body, headers={"Content-Type":"application/json","X-Hexnet-Mqtt-Secret":"x-hexnet-2026-secret"}, method="POST")
try:
  with urllib.request.urlopen(req2, timeout=20) as r:
    print("simulate", r.status, r.read().decode()[:1500])
except Exception as e:
  print("simulate err", e)
  if hasattr(e,'read'):
    print(e.read().decode()[:1500])
