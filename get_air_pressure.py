
import requests
import geocoder

def get_air_pressure(api_key, latitude, longitude):
    base_url = "http://api.openweathermap.org/data/2.5/weather?"
    query_params = f"lat={latitude}&lon={longitude}&appid={api_key}"

    try:
        response = requests.get(base_url + query_params)
        data = response.json()

        if response.status_code == 200:
            air_pressure = data["main"]["pressure"]
            return air_pressure
        else:
            print(f"Error: Unable to retrieve data. Status code: {response.status_code}")
            return None
    except requests.RequestException as e:
        print(f"Error: {e}")
        return None

def get_current_location():
    g = geocoder.ip('me')
    if g.ok:
        return g.latlng
    else:
        print("Error: Unable to fetch current location.")
        return None

def main():
    # Replace 'YOUR_API_KEY' with your actual OpenWeatherMap API key
    api_key = "YOUR_API_KEY"

    # Get current location's latitude and longitude
    current_location = get_current_location()
    if current_location is None:
        return

    latitude, longitude = current_location

    air_pressure = get_air_pressure(api_key, latitude, longitude)

    if air_pressure is not None:
        print(f"Air Pressure: {air_pressure} hPa")

if __name__ == "__main__":
    main()
