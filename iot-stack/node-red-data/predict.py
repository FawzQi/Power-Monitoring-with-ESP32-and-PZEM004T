import sys
import json
from joblib import load

model = load("/data/model_pzem.joblib")

# sys.argv[1] = json string dari Node-RED
data = json.loads(sys.argv[1])

voltage = data["voltage"]
current = data["current"]
power = data["power"]
energy = data["energy"]
frequency = data["frequency"]
pf = data["pf"]

X = [[voltage, current, power, energy, frequency, pf]]
pred = model.predict(X)[0]

print(pred)
