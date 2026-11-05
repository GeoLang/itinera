use std::time::{SystemTime, UNIX_EPOCH};

use axum::Router;
use axum::body::Body;
use axum::http::{Method, Request, StatusCode, header};
use itinera_graph::{Graph, SpeedProfile};
use itinera_server::{AppState, router};
use jsonwebtoken::{EncodingKey, Header, encode};
use serde_json::{Value, json};
use tower::ServiceExt;

const JWT_SECRET: &str = "test-secret";
const NEAREST_URI: &str = "/nearest?lat=0&lon=0";

async fn status(app: Router, method: Method, uri: &str, bearer_token: Option<&str>) -> StatusCode {
    let mut request = Request::builder().method(method).uri(uri);
    if let Some(token) = bearer_token {
        request = request.header(header::AUTHORIZATION, format!("Bearer {token}"));
    }
    app.oneshot(request.body(Body::empty()).unwrap())
        .await
        .unwrap()
        .status()
}

fn signed_token(claims: &Value) -> String {
    encode(
        &Header::default(),
        claims,
        &EncodingKey::from_secret(JWT_SECRET.as_bytes()),
    )
    .unwrap()
}

#[tokio::test]
async fn itinera_jwt_secret_needs_a_role_token_on_every_route_except_the_probes() {
    // ITINERA_JWT_SECRET is process-wide and this binary has one test
    unsafe { std::env::set_var("ITINERA_JWT_SECRET", JWT_SECRET) };
    let app = router(AppState::new(
        Graph::build(Vec::new(), Vec::new()),
        SpeedProfile::car(),
    ));

    let closed_routes = [
        (Method::GET, "/route"),
        (Method::GET, NEAREST_URI),
        (Method::GET, "/isochrone"),
        (Method::POST, "/match"),
        (Method::POST, "/delivery/optimize"),
        (Method::POST, "/network/components"),
        (Method::POST, "/network/od-matrix"),
        (Method::POST, "/network/closest-facility"),
        (Method::POST, "/network/betweenness"),
    ];
    for (method, uri) in closed_routes {
        assert_eq!(
            status(app.clone(), method, uri, None).await,
            StatusCode::UNAUTHORIZED,
            "{uri}"
        );
    }

    let probes = [
        ("/health", StatusCode::OK),
        ("/healthz", StatusCode::OK),
        // readiness_handler returns 503 on an empty graph
        ("/readyz", StatusCode::SERVICE_UNAVAILABLE),
        ("/metrics", StatusCode::OK),
    ];
    for (uri, expected_status) in probes {
        assert_eq!(
            status(app.clone(), Method::GET, uri, None).await,
            expected_status,
            "{uri}"
        );
    }

    let expiry = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs()
        + 300;
    let token_with_role = signed_token(&json!({"sub": "user-1", "exp": expiry, "role": "viewer"}));
    // nearest_handler returns 400 on an empty graph
    assert_eq!(
        status(
            app.clone(),
            Method::GET,
            NEAREST_URI,
            Some(&token_with_role)
        )
        .await,
        StatusCode::BAD_REQUEST
    );
    let token_without_role = signed_token(&json!({"sub": "user-1", "exp": expiry}));
    assert_eq!(
        status(app, Method::GET, NEAREST_URI, Some(&token_without_role)).await,
        StatusCode::UNAUTHORIZED
    );
}
