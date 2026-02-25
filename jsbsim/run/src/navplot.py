import pandas as pd
import plotly.express as px
import plotly.io as pio 


def plot_nav_data(pnt_list):
    
    lats = [ p.lat for p in pnt_list ]
    lons = [ p.lon for p in pnt_list ]
    
    df = pd.DataFrame({
        "lat": lats,
        "lon": lons
    })

    fig = px.scatter_map(df, 
                        lat="lat", 
                        lon="lon",
                        zoom=15.99, 
                        height=600)
    fig.update_layout(map_style="satellite")

    fig.show()
    #pio.write_image(fig, "ljms.png") 