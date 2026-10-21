using Godot;

namespace HakoniwaShip;

/// <summary>
/// エントリシーンのルート。HUD 表示のみを担当する。
/// 船隊生成・箱庭接続・起動モード（--smoke / --hako / --sync-test）は Session（ship_viewer_session.gd）が持つ。
/// 座標系の規約（箱庭 FLU → Godot: x→-Z, y→-X, z→+Y）は ship_fleet.gd の FLU_TO_GODOT に集約。
/// </summary>
public partial class ShipViewerMain : Node3D
{
    private Label _hud;
    private Node _session;

    public override void _Ready()
    {
        _hud = GetNodeOrNull<Label>("Hud/Status");
        _session = GetNodeOrNull("Session");
        GD.Print($"hakoniwa-godot-ship: Godot {Engine.GetVersionInfo()["string"]}, renderer={RenderingServer.GetCurrentRenderingMethod()}");
    }

    public override void _Process(double delta)
    {
        if (_hud == null)
        {
            return;
        }
        var status = _session != null ? (string)_session.Get("status_text") : "no session";
        _hud.Text = $"hakoniwa-godot-ship  |  FPS {Engine.GetFramesPerSecond():0}  |  {status}";
    }
}
