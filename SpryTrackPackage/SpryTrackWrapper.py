import atracsys.stk as tracking_sdk
import  os
from std_msgs.msg import Float32MultiArray
class SpryTrack:
    def __init__(self,geometry_lists):
        self.tracker = tracking_sdk.TrackingSystem()
        if self.tracker.initialise() != tracking_sdk.Status.Ok:
            self.exit_with_error(
                "Error, can't initialise the atracsys SDK api.", self.tracker)

        if self.tracker.enumerate_devices() != tracking_sdk.Status.Ok:
            self.exit_with_error("Error, can't enumerate devices.", self.tracker)

        self.lastFrame = tracking_sdk.FrameData()

        if self.tracker.create_frame(False, 10, 20, 20, 10) != tracking_sdk.Status.Ok:
            self.exit_with_error("Error, can't create frame object.", self.tracker)

        answer = self.tracker.get_enumerated_devices()
        if answer[0] != tracking_sdk.Status.Ok:
            self.exit_with_error("Error, can't get list of enumerated devices", self.tracker)

        print("Tracker with serial ID {0} detected".format(
            hex(self.tracker.get_enumerated_devices()[1][0].serial_number)))

        answer = self.tracker.get_data_option("Data Directory")
        if answer[0] != tracking_sdk.Status.Ok:
            self.exit_with_error("Error, can't read 'Data Directory' option", self.tracker)

        geometry_path = answer[1]

        for geometry in geometry_lists:
            if self.tracker.set_geometry(os.path.join(geometry_path, geometry)) != tracking_sdk.Status.Ok:
                self.exit_with_error("Error, can't create frame object.", self.tracker)
        self.detected=[]
        self.id_to_names={
            580: "HoloLens",
            680: "NewSaw",
            780: "Saw",
            880:"Femur",
            980:"Pelvis",
            2080:"Femur",
            1380:"Arthrex",
            1280:"Clarius",
        }
        self.tracked_objects=[]
        self.poses={}
        # self.tracker.set_int_option(10,3414)
        # self.tracker.set_int_option(11,202)
        self.tracker.set_float_option(10001,2.5)

    def exit_with_error(error, tracking_system):
        print(error)
        answer = tracking_system.get_last_error()
        if answer[0] == tracking_sdk.Status.Ok:
            errors_dict = answer[1]
            for level in ['errors', 'warnings', 'messages']:
                if level in errors_dict:
                    print(errors_dict[level])
    # def get_last_frame(self):
    #     self.tracker.get_last_frame(self.lastFrame)

    def publish_last_frame(self,publishers):
        self.tracker.get_last_frame(self.lastFrame)
        # self.get_last_frame()
        self.message=str(len(self.lastFrame.markers))+"\n"
        self.detected=[]
        for marker in self.lastFrame.markers:
            data=[
                marker.rotation[0][0],marker.rotation[0][1],marker.rotation[0][2],marker.position[0],
                marker.rotation[1][0],marker.rotation[1][1],marker.rotation[1][2],marker.position[1],
                marker.rotation[2][0],marker.rotation[2][1],marker.rotation[2][2],marker.position[2],
                0,0,0,1
            ]
            msg=Float32MultiArray(data=data)
            device=self.id_to_names[marker.geometry_id]
            self.detected.append(device)
            if device=='HoloLens':
                publishers.HoloLens.publish(msg)
            elif device=='Femur':
                publishers.Femur.publish(msg)
            elif device== 'Saw':
                publishers.Saw.publish(msg)
            elif device=='Pelvis':
                publishers.Pelvis.publish(msg)
            elif device=='Arthrex':
                publishers.Arthrex.publish(msg)
            elif device=='Clarius':
                publishers.Clarius.publish(msg)

if __name__=='__main__':
    geometry_lists=['D:\spryTrack SDK x64\data\geometry580.ini',
                    'D:\spryTrack SDK x64\data\geometry780.ini',
                    'D:\spryTrack SDK x64\data\geometry880.ini',
                    'D:\spryTrack SDK x64\data\geometry980.ini',
                    'D:\spryTrack SDK x64\data\geometry1380.ini']
    spryTrack=SpryTrack(geometry_lists)
    while True:
        spryTrack.construct_message()
        # spryTrack.get_last_frame()
        # for marker in spryTrack.lastFrame.markers:
        #     print(marker.geometry_id, marker.position[0], marker.position[1],
        #                            marker.position[2]
        #             , marker.rotation[0][0], marker.rotation[0][1], marker.rotation[0][2]
        #             , marker.rotation[1][0], marker.rotation[1][1], marker.rotation[1][2]
        #             , marker.rotation[2][0], marker.rotation[2][1], marker.rotation[2][2])