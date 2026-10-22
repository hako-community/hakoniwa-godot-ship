using Godot;

namespace HakoniwaShip;

/// <summary>
/// エントリシーンのルート。HUD 表示のみを担当する（状態行と、Overlays の CPA/TCPA/OZT 一覧）。
/// 船隊生成・箱庭接続・起動モード（--smoke / --hako / --sync-test）は Session（ship_viewer_session.gd）が持つ。
/// 座標系の規約（箱庭 FLU → Godot: x→-Z, y→-X, z→+Y）は ship_fleet.gd の FLU_TO_GODOT に集約。
/// </summary>
public partial class ShipViewerMain : Node3D
{
    private Label _hud;
    private Label _targets;
    private Node _session;
    private Node _overlays;

    public override void _Ready()
    {
        _hud = GetNodeOrNull<Label>("Hud/Status");
        _session = GetNodeOrNull("Session");
        _targets = GetNodeOrNull<Label>("Hud/Targets");
        _overlays = GetNodeOrNull("Overlays");
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
        if (_targets != null && _overlays != null)
        {
            _targets.Text = (string)_overlays.Get("summary_text");
        }
    }
}
