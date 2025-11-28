#Tripteron Kinematics & Static Force Simulation

โปรเจกต์นี้เป็นการพัฒนาแบบจำลองเชิงคิเนแมติกส์ (Kinematics) และแรงสถิต (Static Force/Torque)  
ของหุ่นยนต์แบบ Parallel Manipulator ชนิด Tripteron โดยใช้โปรแกรม MATLAB  
ประกอบด้วยการพัฒนา Inverse Kinematics, Forward Kinematics, Jacobian Matrix  
และการนำมาจำลองการเคลื่อนที่ของ End-Effector พร้อมแสดงผลแบบ Animation

รายละเอียดวิธีการทฤษฎีและสมการทั้งหมดสามารถดูได้จากไฟล์รายงาน .pdf  
ส่วน README นี้สรุปเฉพาะภาพรวมโค้ดและวิธีใช้งาน

#โครงสร้างไฟล์ (File Structure)

**1. Kinematics Functions**
ฟังก์ชันที่เกี่ยวข้องกับ IK และ FK
- `forwardx.m` — Forward Kinematics ของขา X  
- `fowardy.m` — Forward Kinematics ของขา Y  
- `fowardz.m` — Forward Kinematics ของขา Z  
- `Invertq1.m` — คำนวณ Joint 1  
- `Invertq2.m` — คำนวณ Joint 2  
- `Invertq3.m` — คำนวณ Joint 3

**2. Denavit–Hartenberg (DH) Transform**
ใช้สร้าง Modified DH-Transformation  
- `DHX.m`  
- `DHY.m`  
- `DHZ.m`

**3. Jacobian Matrix**  
Full Jacobian (6×4) ของแต่ละขา สำหรับคำนวณ Static Torque  
- `jacobian_X_full.m`  
- `jacobian_Y_full.m`  
- `jacobian_Z_full.m`

**4. Simulation Scripts**  
สคริปต์สำหรับจำลองและแสดงผล  
- `simulate.m` — ไฟล์หลักสำหรับการจำลอง  
- `simulate2.m` — เวอร์ชันทดสอบเพิ่มเติม  
- `test.m` — สำหรับตรวจสอบฟังก์ชันเบื้องต้น  
- `drawFrame.m` — สร้างแกน (Frame) ของจุดที่ต้องการ

**5. เอกสารประกอบ**
- `file 2025-FRA333-G16-proposal.pdf` — รายงานหลัก
- `Kinematic_Proposal_Template.pdf`

#การทดสอบ
ดาวโหลดไฟล์แล้วเปิด floder ในโปรแกรม MATLAB
เปิดไฟล์ simulate2.m แล้วรัน

