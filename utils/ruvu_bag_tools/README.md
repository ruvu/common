<!--
Copyright 2020 RUVU Robotics B.V.
-->

# Tools for processing rosbags
## bag_to_json
This tool can be used to convert rosbag files into json. It will skip bag files that are already processed.
```
usage: bag_to_json [-h] [--topics [TOPICS [TOPICS ...]]] infiles [infiles ...]
```
Example output:
```js
[
	{
		"topic": "topic name",
		"timestamp": 3.14,
		"message": {}
	}
]
```
The json schema is as follows:
```js
{
	"definitions": {},
	"$schema": "http://json-schema.org/draft-07/schema#",
	"$id": "http://example.com/root.json",
	"type": "array",
	"title": "The Root Schema",
	"items": {
		"$id": "#/items",
		"type": "object",
		"title": "The messages Schema",
		"required": [
			"topic",
			"timestamp",
			"message"
		],
		"properties": {
			"topic": {
				"$id": "#/items/properties/topic",
				"type": "string",
				"title": "The Topic Schema"
			},
			"timestamp": {
				"$id": "#/items/properties/timestamp",
				"type": "number",
				"title": "The Timestamp Schema",
				"description": "Seconds since epoch"
			},
			"message": {
				"$id": "#/items/properties/message",
				"type": "object",
				"title": "The Message Schema"
			}
		}
	}
}
```
