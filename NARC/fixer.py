import glob
import re

def fixhex(matchobj):
    if ">" in matchobj.group(2):
        return "#{0}{1}{2}".format(matchobj.group(1), matchobj.group(2), matchobj.group(3))
    else:
        return "0{0}H{1}".format(matchobj.group(2), matchobj.group(3))

hexmatcher = re.compile("(>)([0-9A-Fa-f]{1,8})(,?)")

asmfiles = glob.glob("*.ASM")
tblfiles = glob.glob("*.TBL")
macfiles = glob.glob("*.MAC")
hdrfiles = glob.glob("*.H")
incfiles = glob.glob("*.INC")

for asmfile in (asmfiles + tblfiles + macfiles + hdrfiles + incfiles):
    print(asmfile)
    f = open("old/{0}".format(asmfile), "r")
    out = open("{0}".format(asmfile), "w")
    print(f)
    lines = f.readlines()
    print("lines count: {0}".format(len(lines)))
    
    for line in lines:
        hexliterals = re.findall(hexmatcher, line)
    
        # Check for .FILE directive, update to 6.1 format
        if ".FILE" in line:
            out.write(line.replace("'", "\""))          # GSPA 6.10
        
        elif ".TITLE" in line:
            out.write(line.replace("'", "\""))          # GSPA 6.10
            
        elif "$END" in line and "$ENDM" not in line and "$ENDIF" not in line:
            out.write(line.replace("$END", "$ENDM"))    # GSPA 6.10
            
        elif "INIT_M" in line or "S_CHAR" in line:
            out.write(line.replace("\"", "'"))          # GSPA 6.10
            
        # We don't have a VIDEO folder in our global PATH. Make it relative to the narc repo.
        elif "\"\\video\\" in line.lower():
            out.write(line.lower().replace("\"\\video\\", "\"..\\sys\\"))
        elif "\\video\\" in line.lower():
            out.write(line.lower().replace("\\video\\", "..\\sys\\"))
        elif "video\\" in line.lower():
            out.write(line.lower().replace("video\\", "..\\sys\\"))
        
        # Check for old-style hex literals, convert to new-style.
        elif len(hexliterals) > 0:
            #print(re.findall(hexmatcher, line))
            out.write(re.sub(hexmatcher, fixhex, line)) # GSPA 6.10
            
        else:
            out.write(line)
        
        
    out.flush()

