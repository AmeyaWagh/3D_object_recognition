# import rospy
# class bcolors:
#     color_val = {

#         "HEADER" : '\033[95m',
#         "OKBLUE" : '\033[94m',
#         "OKGREEN" : '\033[92m',
#         "WARNING" : '\033[93m',
#         "FAIL" : '\033[91m',
#         "ENDC" : '\033[0m',
#         "BOLD" : '\033[1m',
#         "UNDERLINE" : '\033[4m'
#     }

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
# def tprint(*MSG,**kwargs):
#     _type = "OKGREEN"
#     if "type" in kwargs.keys():
#         _type = kwargs["type"] 
#     print(bcolors.color_val[_type])    
#     for msg in MSG:
#         print(msg)    
#     print(bcolors.color_val["ENDC"])

# def setColor(_type="WARNING"):
#     print(bcolors.color_val[_type])
            
# def clrColor():
#     print(bcolors.color_val["ENDC"])

# def tprintStr(_str,**kwargs):
#     _type = "OKGREEN" 
#     if "type" in kwargs.keys():
#         _type = kwargs["type"]
#     return bcolors.color_val[_type] + _str + bcolors.color_val["ENDC"]        