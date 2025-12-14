import numpy as np
from flask import Flask, request, jsonify
from joblib import load
import sys

app = Flask(__name__)

# Load model yang baru Anda train
# Pastikan file model_pzem.joblib sudah dicopy ke folder /data di Docker
path_model = 'model_pzem.joblib' 
print(f"Sedang meload model dari {path_model}...", flush=True)

try:
    model = load(path_model)
    print("Model BERHASIL diload!", flush=True)
except Exception as e:
    print(f"ERROR Fatal saat load model: {e}", flush=True)
    sys.exit(1)

@app.route('/predict', methods=['POST'])
def predict():
    try:
        content = request.json
        
        # Ambil data input
        # Harapannya berupa list: [voltage, current, power, energy, frequency, pf]
        data_input = content['sensor_data']
        # hilangkan voltage, energy, frequency
        data_input = [data_input[1], data_input[2], data_input[5]]
        
        # Validasi jumlah fitur (harus 6)
        # if len(data_input) != 6:
        #     return jsonify({'status': 'error', 'message': f'Harus 6 fitur, diterima {len(data_input)}'}), 400

        # Prediksi
        hasil = model.predict([data_input])
        # print hasil dan confidence
        print(hasil[0]) 
        print(np.round(model.predict_proba([data_input]), 2))

        return jsonify({
            'status': 'success',
            'prediksi': hasil[0], # Mengembalikan label kelas (0/1/dst) 
        })

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  
