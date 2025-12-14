import pandas as pd
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from joblib import dump
from sklearn.metrics import classification_report


# Load dataset
df = pd.read_csv("dataset.csv")

# Features
X = df[["voltage","current","power","energy","frequency","pf"]]
# drop kolom vltage energy and frequency
X = df[["current","power","pf"]]
# Label
y = df["label"]

# Train/test split
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

# Model Gradient Boosting
model = GradientBoostingClassifier()
model.fit(X_train, y_train)

# Akurasi test
pred = model.predict(X_test)
print("Accuracy:", accuracy_score(y_test, pred))
print(classification_report(y_test, pred))

# Save model
dump(model, "model_pzem.joblib")
print("Model saved as model_pzem.joblib")
