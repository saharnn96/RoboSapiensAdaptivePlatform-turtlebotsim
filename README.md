# RoboSapiensAdaptivePlatform-turtlebotsim

Download the EMQX from the link : https://www.emqx.io/get-started

# RUN mqtt broker 

```bash 
sudo systemctl start emqx
```
## Example mqtt payload
{
    "_ID": "adaptationtype",
    "_UUID": "tbd",
    "_description": "SPEED ADAPTATION",
    "_propertyList": [
        {
            "description": "Slow down robot",
            "name": "target_speed",
            "value": 5.0
        }
    ]
}