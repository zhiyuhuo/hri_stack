#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import re
import numpy as np
import openpyxl
    
if __name__ == '__main__':
    wb = openpyxl.load_workbook('/home/hri/hri_DATA/raw_dataset.xlsx', use_iterators=True)
    print wb
    first_sheet = wb.get_sheet_names()[0]
    sheet = wb.get_sheet_by_name(first_sheet)
    
    reflisttup = ()
    reflist = []
    last_id = 1 
    t = 0
    vblist = []
    for row in sheet.iter_rows():
        if t > 1:
            this_id = row[0].internal_value
            if this_id == last_id:
                ref = row[5].internal_value
                if ref != 'N/A':
                    reflist.append(ref)
                    vblist.append(ref)
            else:
                reflisttup = reflisttup + (reflist,)
                reflist = []
            last_id = this_id
        t = t + 1
        
    reflisttup = reflisttup + (reflist,)    
    print reflisttup
    print len(reflisttup)
    
    vbset = set(vblist)
    vb = list(vbset)
    print vb
    
    
    mat_co_appear = np.zeros(shape=(len(vb), len(vb)))
    vec_occur = np.zeros(shape=(len(vb),1))
    t = 0
    for reflist in reflisttup:
        reflist2 = reflist
        for ref1 in reflist:
            vec_occur[vb.index(ref1)] = vec_occur[vb.index(ref1)] + 1
            for ref2 in reflist2:
                if ref1 != ref2:
                    mat_co_appear[vb.index(ref1),vb.index(ref2)] = mat_co_appear[vb.index(ref1),vb.index(ref2)] + 1

    print mat_co_appear
    print vec_occur
    
    mat_co_appear = mat_co_appear / vec_occur
    print mat_co_appear
    
    row = mat_co_appear.shape[0]
    col = mat_co_appear.shape[1]
    print (row, col)
    
    grounding_co_appear = ()
    for r in range(row):
        for c in range(col):
            if r != c:
                keyword = vb[r] + '_' + vb[c]
                value = mat_co_appear[c,r]
                print (keyword, value)      
                grounding_co_appear = grounding_co_appear + ((keyword, value),)
                
    print grounding_co_appear
    open('/home/hri/hri_DATA/spatial_language_generation/grounding_co_appear.txt', 'w').write('\n'.join('%s %s' % x for x in grounding_co_appear))  
    
    
