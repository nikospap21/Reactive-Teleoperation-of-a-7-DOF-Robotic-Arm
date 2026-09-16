
================================================================================
Οδηγός Εγκατάστασης, Προσομοίωσης (ROS 2) & Σύνδεσης Χειριστηρίου PS2
================================================================================

--------------------------------------------------------------------------------
1. ΠΡΟΕΤΟΙΜΑΣΙΑ ΠΕΡΙΒΑΛΛΟΝΤΟΣ ROS 2 & ΠΡΟΣΟΜΟΙΩΣΗΣ (WSL2 / Ubuntu)
--------------------------------------------------------------------------------
Εκτελέστε τα παρακάτω βήματα μέσα στο τερματικό του WSL (Ubuntu):

1.1 Δημιουργία και είσοδος στον χώρο εργασίας (workspace):
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

1.2 Λήψη των επίσημων πακέτων για το MyArm:
    git clone https://github.com/elephantrobotics/mycobot_ros2.git

1.3 Build των απαραίτητων πακέτων (απαιτείται το description για τα 3D meshes):
    cd ~/ros2_ws
    colcon build --packages-select mycobot_description myarm_300 --symlink-install

1.4 Φόρτωση περιβάλλοντος:
    source install/setup.bash

1.5 Εκκίνηση προσομοίωσης σε RViz:
    ros2 launch myarm_300 test.launch.py


--------------------------------------------------------------------------------
2. ΣΥΝΔΕΣΗ USB BLUETOOTH DONGLE (PS2 CONTROLLER) ΣΤΟ WSL
--------------------------------------------------------------------------------
Επειδή το WSL από προεπιλογή δεν βλέπει τις USB συσκευές των Windows, απαιτείται 
η χρήση του εργαλείου USBIPD.

Βήμα 2.1: Εγκατάσταση USBIPD στα Windows (Εκτέλεση ΜΙΑ φορά)
- Ανοίξτε Windows PowerShell ως Administrator (Διαχειριστής):
    winget install --interactive --exact dorssel.usbipd-win
- Κάντε επανεκκίνηση του υπολογιστή ή κλείστε και ξανανοίξτε το PowerShell.

Βήμα 2.2: Εγκατάσταση εργαλείων joystick στο WSL (Ubuntu)
- Στο τερματικό του WSL τρέξτε:
    sudo apt update
    sudo apt install -y linux-tools-virtual hwdata udev joystick jstest-gtk
    sudo update-alternatives --install /usr/local/bin/usbip usbip $(ls /usr/lib/linux-tools/*/usbip | tail -n1) 20

Βήμα 2.3: Passthrough του USB Dongle στο WSL
- Συνδέστε το USB dongle του χειριστήριου στον υπολογιστή.
- Ανοίξτε Windows PowerShell ως Administrator:
    usbipd list
  (Εντοπίστε το BUSID της συσκευής, π.χ. "1-4" ή "2-1", με περιγραφή Gamepad/Wireless Receiver).

- Αν είναι η πρώτη φορά που τη συνδέετε:
    usbipd bind --busid <BUSID>

- Συνδέστε τη συσκευή στο WSL:
    usbipd attach --wsl --busid <BUSID>

- Για αποσύνδεση (όταν τελειώσετε):
    usbipd detach --busid <BUSID>


--------------------------------------------------------------------------------
3. ΕΠΑΛΗΘΕΥΣΗ ΑΝΑΓΝΩΡΙΣΗΣ ΤΟΥ ΧΕΙΡΙΣΤΗΡΙΟΥ (WSL)
--------------------------------------------------------------------------------
3.1 Έλεγχος ύπαρξης συσκευής εισόδου:
    ls -l /dev/input/
    (Πρέπει να εμφανίζεται η συσκευή js0 ή/και event*).

3.2 Live έλεγχος πλήκτρων και αξόνων:
    jstest /dev/input/js0
    (Κουνήστε τους μοχλούς και πατήστε κουμπιά για να επαληθεύσετε τη ροή δεδομένων.
     Πατήστε Ctrl+C για έξοδο).


--------------------------------------------------------------------------------
4. ΡΥΘΜΙΣΗ PYTHON DEPENDENCIES ΓΙΑ ΤΟ PROJECT
--------------------------------------------------------------------------------
Μέσα στο repository (με ενεργό Python venv):
    pip install numpy scipy pygame pymycobot pyyaml
