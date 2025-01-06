import React, { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import 'leaflet-draw/dist/leaflet.draw.css';
import 'leaflet-draw';

const MapComponent = ({ onGeofenceSet }) => {
  const mapRef = useRef(null);
  const [map, setMap] = useState(null);
  const [userLocation, setUserLocation] = useState(null);
  const [isStreetView, setIsStreetView] = useState(true);
  const drawnItemsRef = useRef(new L.FeatureGroup());

  // OpenStreetMap Tile Layer
  const streetLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
  });

  // Esri World Imagery Satellite Layer
  const satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    attribution: 'Esri, Maxar, Earthstar Geographics, and the GIS User Community'
  });

  // Geolocation Effect
  useEffect(() => {
    navigator.geolocation.getCurrentPosition(
      (position) => {
        setUserLocation([position.coords.latitude, position.coords.longitude]);
      },
      (error) => {
        console.error('Error getting location:', error);
        setUserLocation([0, 0]);
      }
    );
  }, []);

  // Map Initialization Effect
  useEffect(() => {
    if (userLocation && !map) {
      const newMap = L.map(mapRef.current).setView(userLocation, 18);
      
      // Add street view as default
      streetLayer.addTo(newMap);

      // Update draw control to use polygon instead of rectangle
      const drawControl = new L.Control.Draw({
        draw: {
          rectangle: false,
          circle: false,
          circlemarker: false,
          marker: false,
          polyline: false,
          polygon: {
            allowIntersection: false,
            drawError: {
              color: '#e1e4e8',
              timeout: 1000
            },
            shapeOptions: {
              color: '#4A90E2',
              fillOpacity: 0.2
            },
            showArea: true
          }
        },
        edit: {
          featureGroup: drawnItemsRef.current,
          remove: true
        }
      });

      newMap.addControl(drawControl);
      newMap.addLayer(drawnItemsRef.current);

      // Handle draw events
      newMap.on('draw:created', (e) => {
        drawnItemsRef.current.clearLayers();
        const layer = e.layer;
        drawnItemsRef.current.addLayer(layer);

        // Get coordinates of polygon vertices
        const coordinates = layer.getLatLngs()[0].map(latLng => ({
          lat: latLng.lat,
          lng: latLng.lng
        }));

        onGeofenceSet({
          type: 'polygon',
          coordinates: coordinates
        });
      });

      // Handle edit events
      newMap.on('draw:edited', (e) => {
        const layers = e.layers;
        layers.eachLayer((layer) => {
          const coordinates = layer.getLatLngs()[0].map(latLng => ({
            lat: latLng.lat,
            lng: latLng.lng
          }));

          onGeofenceSet({
            type: 'polygon',
            coordinates: coordinates
          });
        });
      });

      // Handle delete events
      newMap.on('draw:deleted', () => {
        onGeofenceSet(null);
      });

      setMap(newMap);
    }
  }, [userLocation, map, onGeofenceSet]);

  // Toggle Layer Effect
  useEffect(() => {
    if (map) {
      // Remove existing layers
      map.eachLayer((layer) => {
        if (layer instanceof L.TileLayer) {
          map.removeLayer(layer);
        }
      });

      // Add appropriate layer based on view state
      isStreetView ? streetLayer.addTo(map) : satelliteLayer.addTo(map);
    }
  }, [isStreetView, map]);

  // Toggle View Handler
  const toggleMapView = () => {
    setIsStreetView(prev => !prev);
  };

  return (
    <div className="relative">
      <div 
        ref={mapRef} 
        style={{ height: '400px', width: '100%', borderRadius: '0.75rem' }}
        className="shadow-inner"
      />
      <button 
        onClick={toggleMapView}
        className="absolute top-2 right-2 z-[1000] bg-white shadow-md px-3 py-2 rounded-lg text-sm"
      >
        {isStreetView ? 'Satellite View' : 'Street View'}
      </button>
    </div>
  );
};

export default MapComponent;