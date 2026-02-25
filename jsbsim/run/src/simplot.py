
from plum import dispatch
from cycler import cycler
from matplotlib import pyplot as plt

class SimPlot:
        
    def __init__(self, d):
        
        self.fig = None
        self.ax = None
        
        self.__d = d
        self.__y = []

    
    def reset(self): 
        self.__y = []

    
    @dispatch
    def add( self, plotnum:int, y:str, y2=False ):
        
        try:
            obj = getattr( self.__d, y )
        except:            
            txt = f"Error::Plot:: Cannot read {y} parameter. Avail are: \n"
            for x in dir(self.__d):
                if not x.startswith("_"):
                    txt += x + "\n"
            raise RuntimeError(txt)

        dct = {
            "ax": plotnum,
            "y": obj.all(),
            "label": y,
            "isy2": y2,
        }
        
        self.__y.append( dct )

    
    def plot(self):
        
        numplots = 0 
        
        for dct in self.__y:
            if (dct["ax"]+1) > numplots:
                numplots = (dct["ax"]+1)

        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
        clr1 = colors[0:5]
        clr2 = colors[5:10]        
        
        fig, ax = plt.subplots( numplots )
        
        try:
            ax[0]
        except:
            ax = [ ax ]
         
        ax2 = [ None for i in range(len(ax)) ]
        
        for pdct in self.__y:
            
            i = pdct["ax"]
            
            if not pdct["isy2"]:
                
                ax[i].plot( self.__d.trel.all(), 
                            pdct["y"], 
                            label=pdct["label"],
                            color=clr1[len(ax[i].lines)] )
            
            else:
                
                if ax2[i] is None:
                    ax2[i] = ax[i].twinx()
                    
                ax2[i].plot( self.__d.trel.all(), 
                             pdct["y"], 
                             label=pdct["label"],
                             color=clr2[len(ax2[i].lines)] )
            

                    
        for i in range(len(ax)):
            ax[i].grid()
            ax[i].set_prop_cycle(cycler('color', ['c', 'm', 'y', 'k']))
            ax[i].legend(loc="upper left")
            ax[i].sharex( ax[0] )
            
            try:
                ax2[i].set_prop_cycle(cycler('color', ['r', 'r', 'g', 'y']))
                ax2[i].legend(loc="upper right")
            except: pass
            
      
        
        plt.show() 
