# Python program to print
# colored text and background
class colors:
    '''
    Colors class:reset all colors with colors.reset; two 
    sub classes fg for foreground 
    and bg for background; use as colors.subclass.colorname.
    i.e. colors.fg.red or colors.bg.greenalso, the generic bold, disable, 
    underline, reverse, strike through,
    and invisible work with the main class i.e. colors.bold
    '''
    reset='\033[0m'
    bold='\033[01m'
    disable='\033[02m'
    underline='\033[04m'
    reverse='\033[07m'
    strikethrough='\033[09m'
    invisible='\033[08m'
    class fg:
        black='\033[30m'
        red='\033[31m'
        green='\033[32m'
        orange='\033[33m'
        blue='\033[34m'
        purple='\033[35m'
        cyan='\033[36m'
        lightgrey='\033[37m'
        darkgrey='\033[90m'
        lightred='\033[91m'
        lightgreen='\033[92m'
        yellow='\033[93m'
        lightblue='\033[94m'
        pink='\033[95m'
        lightcyan='\033[96m'
    class bg:
        black='\033[40m'
        red='\033[41m'
        green='\033[42m'
        orange='\033[43m'
        blue='\033[44m'
        purple='\033[45m'
        cyan='\033[46m'
        lightgrey='\033[47m'
 

def printRed(skk): print("\033[91m {}\033[00m" .format(skk))
def printGreen(skk): print("\033[92m {}\033[00m" .format(skk))
def printYellow(skk): print("\033[93m {}\033[00m" .format(skk))
def printBlue(skk): print("\033[94m {}\033[00m" .format(skk))
def printPurple(skk): print("\033[95m {}\033[00m" .format(skk))
def printCyan(skk): print("\033[96m {}\033[00m" .format(skk))
def printOrange(skk): print("\033[33m {}\033[00m" .format(skk))
def printPink(skk): print("\033[95m {}\033[00m" .format(skk))
