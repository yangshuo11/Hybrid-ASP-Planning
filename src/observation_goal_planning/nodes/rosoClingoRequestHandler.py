class ROSoClingoRequestHandler:
    def __init__(self,request_ids):
        self.pending = []
        self.active = {}
        for id in request_ids:
            self.active[str(id)] = "free"

    def __get_key(self,request):
        for key in self.active.keys():
            if type(self.active[key]) == type(request):
                if self.active[key] == request:
                    return key
        return None

    def add(self,request):
        key = self.__get_key("free")
        if key is not None:
            self.active[key] = request
        else:
            self.pending.append(request)
        return key

    def get(self):
        if self.pending != []:
            key = self.__get_key("done")        
            if key is not None:
                self.active[key] = self.pending.pop()
                return key
        return None

    def cancel(self,request):
        key = self.__get_request_key(request)
        if key is None:
            self.pending.remove(request)
            request.set_canceled()
        return key

    def status_update(self,key,status):
        if type(self.active[key]) != type(""):
            currentStatus = self.active[key].get_goal_status().status
            if status == "accepted" and currentStatus in [0, 7]:
                self.active[key].set_accepted()
            elif status == "rejected" and currentStatus in [0, 7]:
                self.active[key].set_rejected()
                self.active[key] = "done"
            elif status == "succeeded" and currentStatus in [1, 6]:
                self.active[key].set_succeeded()
                self.active[key] = "done"
            elif status == "canceled" and currentStatus in [7, 6]:
                self.active[key].set_canceled()
                self.active[key] = "done"
            elif status == "aborted" and currentStatus in [1, 6]:
                self.active[key].set_aborted()
                self.active[key] = "done"

    def update(self):
        for key in self.active.keys():
            if str(self.active[key]) == "done":
                self.active[key] = "free"

    def get_request(self,id):
        if id in self.active.keys():
            return self.active[id]
        else:
            return None

    def get_information(self,id):
        if id in self.active.keys():
#            return GetGoalInformationResponse(self.active[id].get_goal().information)
            return self.active[id].get_goal().information
        else:
            return None
