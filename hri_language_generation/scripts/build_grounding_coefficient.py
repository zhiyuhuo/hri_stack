#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import re
import numpy as np
import openpyxl
    
if __name__ == '__main__':
    wb = openpyxl.load_workbook('/home/hri/hri_DATA/raw_dataset.xlsx')
    print wb
    sheet = wb.get_sheet_by_name('Sheet1')
    print sheet.cell(row=1, column=2).value
    
