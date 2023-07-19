classdef Task<handle
   properties
      StartLocation 
      EndLocation
      TaskStatus
      AssignedTo
      WorkUnits
   end
   methods
       function obj = Task(Start,End,WorkUnits)
        obj.StartLocation = Start;
        obj.EndLocation = End;
        obj.TaskStatus='UnAssigned';
        obj.WorkUnits = WorkUnits;
       end
   end
end