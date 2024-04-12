import geopandas as gpd
import requests

def get_flight_limit(area):
    if "red" in area.lower():
        return 0
    elif "orange" in area.lower():
        return 25
    elif "yellow" in area.lower():
        return 45
    elif "blue" in area.lower():
        return 60
    elif "green" in area.lower():
        return 120
    else:
        return -1

try:
    gpd.io.file.fiona.drvsupport.supported_drivers['KML'] = 'rw'
except:
    pass

restriction_areas = gpd.read_file('flight_restriction_areas.kml', driver='KML')
restriction_areas = restriction_areas.drop(columns=['Description'])

limits = []
for area in restriction_areas["Name"]:
    limits.append(get_flight_limit(area))

restriction_areas["flight_limit"] = limits

# Parse the points coordinates to make a string for the request to open elevation API
# def parse_points(points):
#     coords = []
#     msg = []

#     for p in points:
#         coords.append(list(p.coords))

#     for i in range(len(coords)):
#         coords[i] = [(x[1], x[0]) for x in coords[i]]
    
#     for c in coords:
#         msg.append(str(c[0][0]) + ',' + str(c[0][1]))
    
#     msg = '|'.join(msg)

#     return msg

def get_elevation(coords):
    api_url = 'https://api.open-elevation.com/api/v1/lookup?locations='
    url = api_url + coords
    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        if 'results' in data and len(data['results']) > 0:
            # Get all elevations and return
            # elevations = []
            # for result in data['results']:
            #     elevations.append(result['elevation'])
            return data
        else:
            print("No results found for the provided coordinates.")
    else:
        print("Error occurred while fetching data:", response.status_code)
