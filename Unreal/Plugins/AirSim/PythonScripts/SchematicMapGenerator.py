import json
import time
import unreal_engine as ue
from unreal_engine.classes import Object, Blueprint, Actor, PlayerStart, StaticMeshActor, RoadBeaconStaticMeshActor, SceneComponent

# Idő kiírása
localtime = time.asctime(time.localtime(time.time()))
print(" ")
print(" ")
print(f"---- LOCAL TIME: {localtime} ----")
print(" ")
print(" ")


# Kimenet készítéshez alapstruktúra
jsonOutputRoot = {
	'nodes': [],
	'edgeSet': [],
}


class RoadBeacon:
	def __init__(self, Id, x, y, connections):
		self.Id = Id
		self.x = x
		self.y = y
		self.connections = list(filter(lambda a: a != 0, connections))

	def __str__(self):
		return f"x: {self.x}, y: {self.y}"

	def to_string_verbose(self):
		return f"Id: {self.Id} {str(self)} connections: {self.connections}"

	def to_json_output(self):
		return {'Position': f"{self.x},{self.y}"}


def round_and_convert_to_int(x):
	return int(round(x, 0))

def convert_absolute_coordinates_to_relative(absoluteX, absoluteY):
	return round_and_convert_to_int(absoluteX / 100 - playerStartX), round_and_convert_to_int(absoluteY / 100 - playerStartY)


# PlayerStart vizsgálat
playerStartCount = len(ue.tobject_iterator(PlayerStart))
if playerStartCount != 1:
	raise Exception(f"A pályán pontosan egy playerStart-nak kell lennie, most {playerStartCount} van.")

for unrealplayerStart in ue.tobject_iterator(PlayerStart):
	playerStartX, playerStartY = round_and_convert_to_int(unrealplayerStart.get_actor_location().x / 100), round_and_convert_to_int(unrealplayerStart.get_actor_location().y / 100)
	#print(f"playerStart: {playerStartX} {playerStartY}")




# Nyers unreal objektumok kigyűjtése által RoadBeacon-ok konstruálása
roadBeaconDictionary = {}
for unrealRoadBeacon in ue.tobject_iterator(RoadBeaconStaticMeshActor):
	parsedTypeName,parsedId = unrealRoadBeacon.get_display_name().split('_')
	if parsedTypeName != 'Road':
		 raise Exception(f"Hibás elnevezés, unrealObjectId: {unrealRoadBeacon.get_display_name()}")

	absoluteX = unrealRoadBeacon.get_actor_location().x
	absoluteY = unrealRoadBeacon.get_actor_location().y
	x, y = convert_absolute_coordinates_to_relative(absoluteX, absoluteY)

	A = int(unrealRoadBeacon.get_property('ConnectionIdA'))
	B = int(unrealRoadBeacon.get_property('ConnectionIdB'))
	C = int(unrealRoadBeacon.get_property('ConnectionIdC'))
	D = int(unrealRoadBeacon.get_property('ConnectionIdD'))

	roadBeacon = RoadBeacon(parsedId, x, y, [A, B, C, D])
	roadBeaconDictionary[int(parsedId)] = roadBeacon
	
	jsonOutputRoot['nodes'].append(roadBeacon.to_json_output())
	print(roadBeacon.to_string_verbose())


# Élek összeválogatása
for roadId, road in roadBeaconDictionary.items():
	for otherRoadId in road.connections:
		if roadId == otherRoadId:
			raise Exception(f"roadId: {roadId} saját önnön magát hivatkozza meg, olyan mint a kígyó, aki a saját önnön farkába harap.")

		if otherRoadId not in roadBeaconDictionary:
			raise Exception(f"roadId: {roadId} meghivatkozza a nemlétező {otherRoadId}-t")

		otherRoad = roadBeaconDictionary[otherRoadId]
		if roadId in otherRoad.connections:
			if not (road.x == otherRoad.x or road.y == otherRoad.y):
				raise Exception(f"a {roadId} - {otherRoadId} nincs egyvonalban, ez a legrövidebb útkereső algoritmusnak számít")

			print(f"Megtalált él: {roadId} - {otherRoadId} között")

			# Új edge hozzáadása
			jsonOutputRoot['edgeSet'].append({
				'From': road.to_json_output(),
				'To': otherRoad.to_json_output(),
			})
			

			# A (dupla) él kiszedése mindkét oldalról, hogy ne járjuk be mindkét oldalról
			otherRoad.connections.remove(roadId)

		else:
			raise Exception(f"roadId: {roadId} és otherRoadId: {otherRoadId} közötti szakasznál hiányzik a(z) {otherRoadId}-ból/ből {roadId}-ba/be vezető él.")
	
	# Szépen végigmentünk rajta hiba nélkül, már teljesen üres
	road.connections = []


# Json kiemenet előállítása
print(json.dumps(jsonOutputRoot, indent=2, separators=(',', ': ')))
