class ClientCommand
{
  
  public:
  enum class Action
  {
      NotDefined,
      Start,
      ExportMetrics,
      Stop,
      Reboot,
      Reset,
      LightOn,
      LightOff,
      SaveSettings,
      AirCirculation

  };

  public:
	  String request ="";
    String body ="";
    String request_header ="";
    String response ="";
    String line;
    String postData = ""; // To store POST data
    String action = "";
    Action Action = Action::NotDefined;
  };