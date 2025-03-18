Yes! You can **download AI/ML packages without using a `requirements.txt` file** by specifying package names directly in the `pip download` command. Here’s how:

---

## **Step 1: Download Packages on the Online System**
Run this command on the **online system** to download all required `.whl` and `.tar.gz` files:

```bash
mkdir ai_ml_packages
pip download numpy pandas matplotlib seaborn scikit-learn scipy joblib xgboost lightgbm opencv-python nltk tqdm -d ai_ml_packages
```

This will download all the packages into the **`ai_ml_packages`** folder.

---

## **Step 2: Transfer to the Offline System**
Copy the **`ai_ml_packages`** folder to your **offline system** using a **USB drive, external hard drive, or LAN transfer**.

---

## **Step 3: Install Packages on the Offline System**
On the **offline system**, navigate to the copied folder and install the packages **without internet**:

```bash
pip install --no-index --find-links=ai_ml_packages *
```

or, if inside the folder:

```bash
pip install --no-index --find-links=. *
```

---

## **Step 4: Verify Installation**
Check if the packages were installed correctly:

```bash
pip list
```

Or test specific packages:

```bash
python -c "import numpy; print(numpy.__version__)"
python -c "import pandas; print(pandas.__version__)"
python -c "import sklearn; print(sklearn.__version__)"
```

---

### **Why This Works?**
✅ No need for a `requirements.txt` file.  
✅ Directly downloads package files for easy transfer.  
✅ Works without internet on the offline system.  

Let me know if you need any modifications! 🚀
